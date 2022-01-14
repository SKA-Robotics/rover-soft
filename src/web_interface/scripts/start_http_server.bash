#!/bin/bash
authbind --deep python3 -m http.server 80 -d $(rospack find web_interface)/website/dist
