# spot-bluelining
Repository for master thesis project about bluelining with Spot

# Contents

The "STL files" folder contains STL-files for all printed parts of the marking device.

The "bluelining" folder found under "bluelining-mt" contains all specific code needed to run the bluelining procedure.

Not included is the spot-sdk, found here: https://github.com/boston-dynamics/spot-sdk, which should be placed in the "blulining-mt" folder

# Procedure

To run the procedure, a computer with two network cards is needed, for example with the use of an extra USB network dongle.

Connect to Spot's WIFI.

Connect to Spot through the terminal according to the official documentation.

Start the data stream from the used tracker and direct it to your computer.

Start the program "bluelining.py".

# Caveats

The goal points are written directly in the "bluelining.py" code as of now.
