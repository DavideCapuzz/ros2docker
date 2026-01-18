#!/bin/bash

# Funtion to extract the list of services form the docker-compose.yml file
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
    # For every service check if it exist the endfunction in the a folder with the name of the
    # service in the top level otherwise copy the template
    #
    if [ -d "../${SERVICE}" ]; then
        echo "Directory ../${SERVICE} exists."
        cp -r ../${SERVICE} ./tmp/
    else
        echo "Directory ../${SERVICE} not exists."
        mkdir -p ./tmp/${SERVICE}
        cp template/endfunction.sh ./tmp/${SERVICE}/endfunction.sh
    fi

done