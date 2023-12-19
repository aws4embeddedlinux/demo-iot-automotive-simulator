#!/bin/bash

# Function to convert candump output to cansend format
convert_to_cansend_format() {
    while read -r line; do
        # Extracting CAN ID and data payload
        # Using cut to extract fields based on specific character positions
        can_id=$(echo "$line" | cut -d' ' -f3)
        data=$(echo "$line" | cut -d' ' -f5- | tr -d '[] ' | sed 's/ //g')

        # Format for cansend
        echo "${can_id}#${data}"
    done
}

# Main pipeline
candump vcan0 | convert_to_cansend_format | socat - UDP4-DATAGRAM:239.255.0.1:3030,reuseaddr
