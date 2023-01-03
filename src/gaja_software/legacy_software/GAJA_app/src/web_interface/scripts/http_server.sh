#!/bin/bash
authbind --deep python3 -m http.server 8060 -d $(rospack find web_interface)/website
