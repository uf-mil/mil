# Battery Charger

The battery charger is used when testing SubjuGator. The charger is used to
charge batteries that are depleted from being used in the sub in the water.

## How to use
1. Check the battery percentage using the *HiTEC LiPo Checker - Voltage Checker
and Equilizer*. Plug the device to the connectors using the balance connectors
(white connectors). If the percentage is > 80%, the battery is charged and no
further action is needed.
2. Power the battery charger by plugging the cable into the wall.
3. Remove the balance connectors from the *HiTec LiPo Checker* and plug in the
balance connectors into the CH1 side the ports of the battery charger. Connect
the anderson connectors (red and black connectors) on the battery into the
anderson connectors on the CH1 side charger. Connect the red wires together
and the black wires together. Note the process is the same with the CH2 side.
4. Press *PHYSTART-1*, then with the center dial, select and press on *LiPo*.
Select *Charge* to charge the battery. Use the default settings to charge.
Select *Yes* on the pop-up.
5. An icon should starting blinking in the upper corner. The battery is
charging and the voltage in each cell of the battery is shown. If configured
with the default settings, the charger will charge until the battery is at 90%
to prevent the issues with over charging.

:::{danger}
During the 2022 RoboSub competition, the CH-2 (right port) of the charger
burned two batteries. Until fixed, do **NOT** use the CH-2 of the charger
until it is fixed.
:::

## Useful Links/Documents
- [Store Page](https://www.progressiverc.com/products/icharger-308duo)
- [Manual](https://github.com/uf-mil-electrical/SubjuGator-Sensor-Actuator-Datasheets/blob/main/SubjuGator8/iCharger_308Duo_Manual.pdf)
