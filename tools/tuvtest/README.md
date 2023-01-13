1. Connect your laptop to your comma device's hotspot and open a new SSH shell. Since service is being run on your laptop, we need to write a parameter to let controlsd know to start in failState debug mode:
   ```shell
   # on your comma device
   echo -n "1" > /data/params/d/FailStateMode
   ```