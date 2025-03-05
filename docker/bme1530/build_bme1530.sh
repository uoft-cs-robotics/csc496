#!/bin/bash
docker-compose -f docker-compose-bme1530-gui.yml down -v --remove-orphans
docker-compose -f docker-compose-bme1530-gui.yml build
docker-compose -f docker-compose-bme1530-gui.yml up -d