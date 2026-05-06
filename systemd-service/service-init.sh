#!/bin/bash

sudo systemctl daemon-reexec
sudo systemctl daemon-reload

sudo systemctl enable av-server
sudo systemctl start av-server