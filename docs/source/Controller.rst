##################
Controller
##################

baseController Class
=====================

The baseController Class has several modules which can be used to simplify the process of setting up controllers

simpleLowPass
  This is a very simple lowpass filter. Output is calculated by

  .. math::

    y_{n} = \alpha * u_{n} + (1 - \alpha) * y_{n-1}


compensateEMF
  force maximum slope change of input. Output is calculated by
  
  .. math::

    y_{n}=\left\{\begin{array}{ll}y_{n} = u_{n},   & x <y_{max} \\
       (u_{n} - y_{n-1} ) y_{max} + y_{n-1} , & else\end{array}\right.


ffControl
  This Module implements a FeedForward Component.

  .. math::

    y_{n} = a * u_{n} + b * sign(u_{n})

pidControl

angleNorm

.. todo:: document baseController



.. todo:: document controller subelements

.. graph:: time

  "Motorboard" -- "RoNex";
