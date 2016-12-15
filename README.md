# AM335x MCASP to ADSP-21262 SPORT driver #

This driver allows to connect ADSP-2161 devices to BeagleBone's AM335x McASP serial interface.

### Running ###

Currently only 3.8.13-bone kernel is supported.

BB-BONE-SPORT cape is required for propper operation. Cape is available as part of [beagle-linux repo] (https://github.com/geotechnologies/bb-linux/tree/mykernel).

```sh
CAPEMGR=/sys/devices/bone_capemgr.*/slots
grep -q BB-BONE-SPORT $CAPEMGR || echo BB-BONE-SPORT > $CAPEMGR

insmod gtsport.ko
```
When inserted data could be accessed via `/dev/gtsport` character device. Currently only `read(2)` and polling is supported.

### TODO ###

* Read DMA descriptor to move buffer's head
* Cleanup McASP initialization
* Update buffer's head with timer
* Port to 4.4 kernel version
