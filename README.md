libraries
=========

libraries directory tree, intended to be placed in sketchbook/libraries (or similar),
for xmega-specific libs.

The files in this directory tree are likely to be derived from the Arduino IDE headers
and implementation for standard Arduino libraries.  To avoid collision, and avoid
problems with software upgrades, you should copy these to your private 'sketchbook'
libraries directory.  Assuming your sketchbook is '~/sketchbook', the contents of
this directory would be copied to '~/sketchbook/libraries/'.

the following libraries are included:

XSPI - SPI library, modified for XMega.  Use '#include <XSPI.h>'
XSD  - SD Card library, modified for XMega.  Use '#include <XSD.h>'


