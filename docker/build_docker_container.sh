docker-compose -f docker-compose-gui.yml down -v --remove-orphans
docker-compose -f docker-compose-gui.yml build --no-cache
docker-compose -f docker-compose-gui.yml up --no-start
