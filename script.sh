#!/usr/bin/env sh
#
# discover_topics.sh — Discover ROS / ROS 2 topics and emit structured JSON.
#
# Usage:
#   ./discover_topics.sh [--hz-window <msgs>] [--hz-timeout <seconds>]
#
# Options:
#   --hz-window   Rolling window size (message count) for rate averaging.  (default: 50)
#   --hz-timeout  Max seconds to wait per topic for messages before
#                 declaring it silent.                                      (default: 5)
#
# Output (stdout): JSON object with all discovered topics and metadata.
# Progress logs go to stderr.

set -euo pipefail

# ─── defaults ────────────────────────────────────────────────────────────────
HZ_WINDOW=50
HZ_TIMEOUT=5

while [[ $# -gt 0 ]]; do
  case "$1" in
    --hz-window)  HZ_WINDOW="$2";  shift 2 ;;
    --hz-timeout) HZ_TIMEOUT="$2"; shift 2 ;;
    -h|--help)    sed -n '/^#/!q;s/^# \?//p' "$0"; exit 0 ;;
    *)            echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

# ─── source ROS ──────────────────────────────────────────────────────────────
if [[ -z "${ROS_VERSION:-}" ]]; then
  for _d in /opt/ros/*/; do
    [[ -f "${_d}setup.bash" ]] && { source "${_d}setup.bash"; break; }
  done
fi

[[ -z "${ROS_VERSION:-}" ]] && { echo '{"error":"No ROS found under /opt/ros/"}' >&2; exit 1; }

ROS_VER="$ROS_VERSION"
DISTRO="${ROS_DISTRO:-unknown}"

[[ "$ROS_VER" != "1" && "$ROS_VER" != "2" ]] && {
  echo "{\"error\":\"Unsupported ROS_VERSION: '$ROS_VER'\"}" >&2; exit 1
}

# ─── JSON helpers ───────────────────────────────────────────────────────────

# Escape a string for embedding inside JSON double-quotes.
json_esc() {
  local s="$1"
  s="${s//\\/\\\\}"
  s="${s//\"/\\\"}"
  s="${s//$'\n'/\\n}"
  s="${s//$'\r'/\\r}"
  s="${s//$'\t'/\\t}"
  printf '%s' "$s"
}

# Produce a JSON-quoted string: "value"
json_str() { printf '"%s"' "$(json_esc "$1")"; }

# Read newline-delimited strings from stdin → ["a", "b"]
# Empty input → []
json_str_array() {
  local out="[" first=true line
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ -z "$line" ]] && continue
    $first && first=false || out+=", "
    out+="$(json_str "$line")"
  done
  printf '%s' "${out}]"
}

# ─── schema parser ──────────────────────────────────────────────────────────
# Parses the indented output of `rosmsg show` (2-space indent) or
# `ros2 interface show --no-comments` (tab indent) into a compact nested
# JSON array of field objects.
#
#   Input (ros2):                     Output (compact):
#     Vector3  linear                 [{"type":"Vector3","name":"linear",
#     \tfloat64 x                       "fields":[{"type":"float64","name":"x"},
#     \tfloat64 y                                 {"type":"float64","name":"y"},
#     \tfloat64 z                                 {"type":"float64","name":"z"}]},
#     Vector3  angular                 {"type":"Vector3","name":"angular",
#     \tfloat64 x                       "fields":[...]}]
#
# Algorithm: process lines one at a time, tracking indent depth with a stack.
# Each field object is opened but NOT closed immediately — if the next line is
# deeper, we append  ,"fields":[  and push. Otherwise we close with } and
# pop back as needed.

parse_schema() {
  local raw="$1"
  local json="["
  local prev_indent=-1 have_prev=false
  # indent_stack: bash indexed array used as a stack
  local -a istack=()

  while IFS= read -r line; do
    # strip trailing inline comments:  "float32 x  # comment" → "float32 x"
    local no_comment="${line%%#*}"
    # strip trailing whitespace
    no_comment="${no_comment%"${no_comment##*[![:space:]]}"}"

    # compute raw content (leading whitespace removed)
    local stripped="${no_comment#"${no_comment%%[![:space:]]*}"}"
    [[ -z "$stripped" ]] && continue

    # count indent: number of leading characters that are whitespace
    local indent=$(( ${#no_comment} - ${#stripped} ))

    # ── identify the field ────────────────────────────────────────────
    local ftype fname fextra="" is_const=false

    if [[ "$stripped" == *"="* ]]; then
      # constant, e.g. "uint8 DEBUG=10"
      is_const=true
      read -r ftype fextra <<< "$stripped"
      fname="${fextra%%=*}"
      fname="${fname%% }"
      local fvalue="${fextra#*=}"
      fvalue="${fvalue# }"
    else
      # regular field, e.g. "float64 x" or "float64 x 0.0"
      read -r ftype fname fextra <<< "$stripped"
      [[ -z "${fname:-}" ]] && continue
    fi

    # ── adjust nesting relative to previous line ─────────────────────
    if $have_prev; then
      if (( indent > prev_indent )); then
        # previous field becomes a parent — open its fields array
        json+=", \"fields\": ["
        istack+=("$prev_indent")
      else
        # close the previous field object
        json+="}"
        # pop back through any deeper levels we've left
        while (( ${#istack[@]} > 0 )); do
          local top="${istack[${#istack[@]}-1]}"
          (( top >= indent )) || break
          json+="]}"
          unset 'istack[${#istack[@]}-1]'
        done
        json+=", "
      fi
    fi

    # ── emit the field (leave the object open — may get children) ────
    json+="{\"type\": $(json_str "$ftype"), \"name\": $(json_str "$fname")"
    if $is_const; then
      json+=", \"value\": $(json_str "$fvalue")"
    elif [[ -n "$fextra" ]]; then
      json+=", \"default\": $(json_str "$fextra")"
    fi

    prev_indent=$indent
    have_prev=true
  done <<< "$raw"

  # ── close everything still open ────────────────────────────────────
  if $have_prev; then
    json+="}"
    while (( ${#istack[@]} > 0 )); do
      json+="]}"
      unset 'istack[${#istack[@]}-1]'
    done
  fi

  printf '%s' "${json}]"
}

# ─── topic listing ──────────────────────────────────────────────────────────

declare -a T_NAMES=()
declare -A T_TYPES=()

if [[ "$ROS_VER" == "1" ]]; then
  # rostopic list -v produces:
  #   Published topics:
  #    * /topic [pkg/Type] N publisher(s)
  #   Subscribed topics:
  #    * /topic [pkg/Type] N subscriber(s)
  while IFS= read -r line; do
    if [[ "$line" =~ ^\ \*\ (/[^ ]+)\ \[([^]]+)\] ]]; then
      local_name="${BASH_REMATCH[1]}"
      local_type="${BASH_REMATCH[2]}"
      if [[ -z "${T_TYPES[$local_name]+x}" ]]; then
        T_NAMES+=("$local_name")
      fi
      T_TYPES["$local_name"]="$local_type"
    fi
  done < <(rostopic list -v 2>/dev/null || true)

  # fallback: plain list + per-topic type lookup
  if [[ ${#T_NAMES[@]} -eq 0 ]]; then
    while IFS= read -r t; do
      t="${t// /}"; [[ -z "$t" ]] && continue
      T_NAMES+=("$t")
      T_TYPES["$t"]=$(rostopic type "$t" 2>/dev/null || echo "unknown")
    done < <(rostopic list 2>/dev/null)
  fi
else
  # ros2 topic list -t produces:
  #   /topic [pkg/msg/Type]
  while IFS= read -r line; do
    if [[ "$line" =~ ^(/[^ ]+)\ \[([^]]+)\] ]]; then
      T_NAMES+=("${BASH_REMATCH[1]}")
      T_TYPES["${BASH_REMATCH[1]}"]="${BASH_REMATCH[2]}"
    elif [[ -n "${line// /}" ]]; then
      T_NAMES+=("${line// /}")
      T_TYPES["${line// /}"]="unknown"
    fi
  done < <(ros2 topic list -t 2>/dev/null || true)
fi

[[ ${#T_NAMES[@]} -eq 0 ]] && { echo '{"error":"No topics found. Is a ROS master/daemon running?"}' >&2; exit 1; }

# ─── data-collection helpers ────────────────────────────────────────────────

get_schema_text() {
  local msg_type="$1"
  if [[ "$ROS_VER" == "1" ]]; then
    rosmsg show "$msg_type" 2>/dev/null || true
  else
    ros2 interface show "$msg_type" --no-comments 2>/dev/null || true
  fi
}

# Populates PUBS_JSON and SUBS_JSON for a given topic.
get_pub_sub() {
  local topic="$1" raw

  if [[ "$ROS_VER" == "1" ]]; then
    raw=$(rostopic info "$topic" 2>/dev/null || true)
    local pubs="" subs="" section=""
    while IFS= read -r line; do
      [[ "$line" == Publishers:* ]]  && { section=pub; continue; }
      [[ "$line" == Subscribers:* ]] && { section=sub; continue; }
      if [[ "$line" =~ ^\ \*\ (/[^ ]+) ]]; then
        local node="${BASH_REMATCH[1]}"
        [[ "$section" == pub ]] && pubs+="${node}"$'\n'
        [[ "$section" == sub ]] && subs+="${node}"$'\n'
      fi
    done <<< "$raw"
    PUBS_JSON=$(printf '%s' "$pubs" | json_str_array)
    SUBS_JSON=$(printf '%s' "$subs" | json_str_array)
  else
    # ros2 topic info -v output format (per endpoint block):
    #   Node name: <name>
    #   Node namespace: <ns>
    #   ...
    #   Endpoint type: PUBLISHER | SUBSCRIPTION
    raw=$(ros2 topic info -v "$topic" 2>/dev/null || true)
    local pubs="" subs="" nname="" nns=""
    while IFS= read -r line; do
      local s="${line#"${line%%[![:space:]]*}"}"
      if [[ "$s" == "Node name: "* ]]; then
        nname="${s#Node name: }"
      elif [[ "$s" == "Node namespace: "* ]]; then
        nns="${s#Node namespace: }"
      elif [[ "$s" == "Endpoint type: "* ]]; then
        local ep="${s#Endpoint type: }"
        local path="${nns:-/}"
        path="${path%/}/${nname:-unknown}"
        [[ "$ep" == PUBLISHER ]]    && pubs+="${path}"$'\n'
        [[ "$ep" == SUBSCRIPTION ]] && subs+="${path}"$'\n'
        nname="" nns=""
      fi
    done <<< "$raw"
    PUBS_JSON=$(printf '%s' "$pubs" | json_str_array)
    SUBS_JSON=$(printf '%s' "$subs" | json_str_array)
  fi
}

measure_hz() {
  local topic="$1" raw rate
  if [[ "$ROS_VER" == "1" ]]; then
    raw=$(PYTHONUNBUFFERED=1 timeout "${HZ_TIMEOUT}s" \
          rostopic hz -w "$HZ_WINDOW" "$topic" 2>&1 || true)
  else
    raw=$(PYTHONUNBUFFERED=1 timeout "${HZ_TIMEOUT}s" \
          ros2 topic hz --window "$HZ_WINDOW" "$topic" 2>&1 || true)
  fi
  # grab the last "average rate: N.NNN" value
  rate=$(echo "$raw" | awk '/average rate:/{r=$3} END{if(r!="") print r}')
  printf '%s' "${rate:-null}"
}

# ─── main: collect and assemble JSON ────────────────────────────────────────

total=${#T_NAMES[@]}
echo "Discovering ${total} topic(s) on ROS ${ROS_VER} (${DISTRO}) ..." >&2
echo "  hz-window=${HZ_WINDOW}  hz-timeout=${HZ_TIMEOUT}s" >&2

topics_json=""
idx=0
for topic in "${T_NAMES[@]}"; do
  idx=$((idx + 1))
  echo "  [${idx}/${total}] ${topic}" >&2

  msg_type="${T_TYPES[$topic]}"
  schema_json=$(parse_schema "$(get_schema_text "$msg_type")")
  get_pub_sub "$topic"
  hz=$(measure_hz "$topic")

  [[ -n "$topics_json" ]] && topics_json+=","
  read -r -d '' entry <<ENTRY || true
    {
      "name": $(json_str "$topic"),
      "message_type": $(json_str "$msg_type"),
      "schema": ${schema_json},
      "publishers": ${PUBS_JSON},
      "subscribers": ${SUBS_JSON},
      "frequency_hz": ${hz}
    }
ENTRY
  topics_json+="${entry}"
done

TS=$(date -u +%Y-%m-%dT%H:%M:%SZ)

cat <<JSONEOF
{
  "ros_version": $(json_str "$ROS_VER"),
  "ros_distro": $(json_str "$DISTRO"),
  "discovered_at": $(json_str "$TS"),
  "settings": {
    "hz_window": ${HZ_WINDOW},
    "hz_timeout_s": ${HZ_TIMEOUT}
  },
  "topic_count": ${total},
  "topics": [${topics_json}
  ]
}
JSONEOF
