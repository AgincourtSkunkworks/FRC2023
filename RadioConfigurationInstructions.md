# How to Load Firmware, Configure, and Flash the Radio
So that future me, or whoever does this next year isn't absolutely clueless.

## Table of Contents
[Documentation](#documentation)  
[Downloading Radio Configuration Utility](#downloading-radio-configuration-utility)  
[Loading Firmware](#loading-firmware)  
[Troubleshooting](#troubleshooting)

## Documentation
FRC themselves provide documentation on radio configuration, which is available [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html). This information is a summary of my (our) own experiences with the radio, however it's recommended you read through that guide as well as this will only contain a summary of the basic steps. A majority of the information in this guide is on troubleshooting the issues we encountered.

## Downloading Radio Configuration Utility
The FRC Radio Configuration tool is needed to configure the radio, the download link at time of writing is [here](https://firstfrc.blob.core.windows.net/frc2023/Radio/FRC_Radio_Configuration_23_0_2.zip), however the most recent link can always be found at the top of the [documentation](#documentation). After the tool is downloaded, unzip it, and use the EXE file to run the tool. Ensure that the tool is run as an Administrator (it should prompt you to do so if you don't) -- it will not work otherwise.

## Configuring the Radio
<!-- TODO: Add Radio Configuration Instructions -->

## Troubleshooting
### Radio is not turning on
Ensure that the POE (Power over Ethernet, a.k.a. the orange Ethernet cable) is connected to the bottom (the one next to the barrel power connector)

### What network interface do I use?
If prompted for a network interface (if you choose the wrong one earlier, you can use the Tools → Network Interface menu to change it), ensure that you have **Ethernet** selected. If **Ethernet** is not available in the list, ensure that the radio is on, an Ethernet cable is plugged in, and the Ethernet adapter is working.

**Do not select Local Area Connection, or anything other than Ethernet. They may seem to work, but they are *incorrect***

### Load Firmware NPF Driver/Name Error
This happens when the configuration tool gets confused about which interface to use. The documentation provides this excerpt on it, however I'll go into more detail later:

> If you see an error about NPF name, try disabling all adapters other than the one being used to program the radio. If only one adapter is found, the tool should attempt to use that one. See the steps in Disabling Network Adapters for more info.

As the documentation says, we'll need to disable all network adapters other than the one we're using. To do this, follow these steps:

1. Press Windows + R to open the Run dialog
2. Type in `control`, and press enter
3. Select Network and Internet
4. Select Network and Sharing Center
5. Click `Change adapter settings` on the left panel
6. For each adapter that is not "Ethernet" (or the one you're using, it may be under a different name):
    1. Right-click the adapter
    2. Select `Disable`
7. Ensure that every single adapter other than the one you're using is disabled. It doesn't matter if it seems like it shouldn't affect anything, it will.
8. Close and reopen the Radio Configuration Tool
9. Try to Load Firmware again

### Verification Issues
<!-- TODO: Add instructions for verification issues -->
