#!/bin/bash
python3 -m http.server 8080 -d $(rospack find interface)/website/dist