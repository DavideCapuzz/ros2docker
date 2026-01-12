#!/bin/bash

get_services() {
  awk '
    /^services:/ { in_services=1; next }
    in_services && /^[^[:space:]]/ { in_services=0 }
    in_services && /^[[:space:]]{2}[a-zA-Z0-9_-]+:/ {
      gsub(":", "", $1)
      print $1
    }
  ' ../docker-compose.yml
}

SERVICES=($(get_services))
echo "${SERVICES[@]}"

for SERVICE in "${SERVICES[@]}"; do
    if [ -d "../${SERVICE}" ]; then
        echo "Directory ../${SERVICE} exists."
        cp -r ../${SERVICE} ./tmp/
    else
        echo "Directory ../${SERVICE} not exists."
        mkdir -p ./tmp/${SERVICE}
        cp template/endfunction.sh ./tmp/${SERVICE}/endfunction.sh
    fi

done