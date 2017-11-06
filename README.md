libraries
=========

libraries directory tree, intended to be placed in sketchbook/libraries (or similar),
for xmega-specific libs.<br>
<br>
The files in this directory tree are likely to be derived from the Arduino IDE headers
and implementation for standard Arduino libraries.  To avoid collision, and avoid
problems with software upgrades, you should copy these to your private 'sketchbook'
libraries directory.  Assuming your sketchbook is '~/sketchbook', the contents of
this directory would be copied to '~/sketchbook/libraries/'.<br>
<br>
the following libraries are included:<br>
<br>
XSPI   - SPI library, modified for XMega.  Use '#include &lt;XSPI.h&gt;'<br>
XSD    - SD Card library, modified for XMega.  Use '#include &lt;XSD.h&gt;'<br>
XWire  - 'Wire' library, modified for XMega.  Use '#include &lt;XWire.h&gt;'<br>
I2CDev - A modified version of the 'I2CDev' library that works with XWire<br>
QDec   - Quadrature decoding library. Modified from the AVR1600 code.<br>
<br>
<br>
Most (if not all) of the libraries are derived from the originals shipped with
Arduino 1.06 and are licensed using [L]GPL v2 or v3.  The same licenses
apply here, and of course (like the originals) they are available to you
free of charge in accordance with the original license terms.<br>
<br>
<br>
The I2CDev library is licensed with an M.I.T. license, and was essentially
'cloned' from the following github location:<br>
<br>
&nbsp;&nbsp;https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev<br>
<br>


