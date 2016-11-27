##################
Installation
##################


Linux
=======
Get your favourite Linux Distro, preferable Debian based.

In doubt use `Ubuntu 14.04 <http://releases.ubuntu.com/14.04/>`_

.. note:: it is important to use Ubuntu 14.04 or 13.10. This is because ROS Indigo - which is required for the RoNex board - only supports those two

Realtime Patch
--------------
Get your `PREEMPT-RT <https://www.kernel.org/pub/linux/kernel/projects/rt/>`_ patch and `Kernel <https://www.kernel.org/pub/linux/kernel/v4.x/>`_ from Kernel.org.

.. note::

  If you have to upgrade your Kernel, make sure your system works with just the vanilla kernel first - just skip the patching part.

Now extract the kernel and patch it:

::

  $ tar vxf linux-<x.x.xx>
  $ cd linux-<x.x.xx>
  $ gzip ../patch-<x.x.xx-rtxx>.patch.gz | patch -p1 --verbose

After sucessfully patching, config it

::

  $ make oldconfig
  $ make menuconfig

Make sure to set
::

  Processor type & features
    --> Preemption Model
      ---> Fully preemptable Kernel


Also disable
::

  CONFIG_CPU_FREQ
  CONFIG_CPU_IDLE
  CONFIG_CC_STACKPROCESSOR
  CONFIG_APM
  CONFIG_ACPI_PROCESSOR
  CONFIG-INTEL_IDLE
  CONFIG_PCI_MSI

i.e. set them to # <> is not set in the .config file that was created.

if you need to debug it and trace events you should also set
::

  CONFIG_WAKEUP_TIMING=y
  CONFIG_LATENCY_TRACE=y
  CONFIG_CRITICAL_PREEMPT_TIMING=y
  CONFIG_CRITICAL_IRQSOFF_TIMING=y


Now compile modules and image and install them
::

  $ sudo make -j4 bzImage modules | egrep -wi --color 'critical|error|warning'
  $ sudo make modules_install
  $ sudo make install

Reboot an pray for it to work out.


Test Patch
--------------

Get RT-Tests and compile them
::

  $ git clone git://git.kernel.org/pub/scm/utils/rt-tests/rt-tests.git --branch stable/v1.0
  $ cd rt-tests
  $ make all

Run tests, e.g.
::

  $ sudo ./cyclictest -a -t -h -n -i 10000 10000

You can find a :doc:`bashscript<cyclictestscript>` for testing

  .. seealso:: XenoMai


.. todo::

  Install PREEMPT-RT



Install ROS Indigo
===================

.. note:: For now RoNex just supports the Indigo version of ROS.

See `ROS Wiki <http://wiki.ros.org/indigo/Installation/Ubuntu>`_


.. todo::

  Install ROS Indigo

Install RoNex driver and dependencies
========================================

See `RoNex Wiki <http://sr-ronex.readthedocs.io/en/latest/General/Setting-up-your-computer.html>`_

.. todo::

  Install ROS packages
