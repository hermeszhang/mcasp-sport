# AM335x MCASP to ADSP-21262 SPORT driver #

This driver allows to connect ADSP-2161 devices to BeagleBone's AM335x McASP serial interface.

### Running ###

You have to load BB-BONE-SPORT cape first:

```sh
CAPEMGR=/sys/devices/bone_capemgr.*/slots
grep -q BB-BONE-SPORT $CAPEMGR || echo BB-BONE-SPORT > $CAPEMGR

insmod gtsport.ko
```

After inserting module would create character deivce `/dev/gtsport`.