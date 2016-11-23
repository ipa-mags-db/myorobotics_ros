##################
Hardware Setup
##################

.. todo:: add pictures

Connectors
==========

RoNex Pin layout
-----------------

(rippled side up)

+----+-----+----+-----+----+-----+------+------+
| 6V | GND | 5V | GND | CS | CLK | MOSI | MISO |
+====+=====+====+=====+====+=====+======+======+
|`-` | `-` | `-`|  x  |  x |  x  |  x   |  x   |
+----+-----+----+-----+----+-----+------+------+

Cables
-------
- use cables with crimp contacts from MYO Robotics toolkit (already pre-crimped)
- Cut off connectors at one side, remove isolation (very short!)
- Crimp cables with Crimp-Tool from E-Lab (MOLEX Picoblade)

.. note:: Test if crimp contacts are actually working!

Connectors
-----------

+------------------+--------------------------+
| Motorboard Side  | MYO Robotics connectors  |
+------------------+--------------------------+
| RoNex Side       | 8-pin *picoblade* system |
+------------------+--------------------------+


Parts overview
-------------------

Part No @ Farnell

RoNex

+--------------------+--------------------+
| Molex Connector    | PartNR: 0510210800 |
+--------------------+--------------------+
| Molex Crimpcontact | PartNR: 0500588000 |
+--------------------+--------------------+
| (Molex housing)    | PartNR: 0530480810 |
+--------------------+--------------------+

Motorboard

.. todo:: add parts for motorboard connector


Configuration
==============
RoNex
------
Should work out of the box
Install process:

.. todo:: add/link installation process for RoNex


Motorboard
-----------

USB
^^^
Use the UART Configuration tool from Myorobotics SVN

CAN
^^^
- Use the CAN Configuration tool from Myorobotics SVN
- make sure dip-switches are in CAN configuration (both set in direction of the border of the board)
- Dont forget to set to SPI mode after configuration! (both set in direction of the middle of the board)
- It will ask for a Node ID – can be anything – try [-1;4], those are the usual ones

ConfigFile
-----------

For the configuration mostly two values are important to get useful values:

+----------------+-------------+
| Operation Mode | -1 for SPI  |
+----------------+-------------+
| Cycle time     |   ca 50     |
+----------------+-------------+

.. todo:: document other values in config
