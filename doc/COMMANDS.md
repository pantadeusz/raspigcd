# Supported commands

## Summary

### General meaning of symbols

| symbol | meaning |
| ------ | ------- |
| G      | G command |
| M      | M command |
| X      | X coordinate in mm, or delay in seconds |
| Y      | Y coordinate in mm |
| Z      | Z coordinate in mm |
| F      | feedrate in mm/s |
| P      | delay in milliseconds |

### The supported commands list

| ? | ? | Meaning           | Coordinates accepted | Example  |
| - | - |:-----------------:| -------------------- | :--------|
| G | 0 | fast move         | X Y Z F              | G0X10Y20F1000 |
| G | 1 | work move         | X Y Z F              | G1X10Y20F1000 |
| G | 1 | work move         | X Y Z F              | G1X10Y20F1000 |
| G | 28 | go home to endstop along axis | X Y Z | G28X0 |
| G | 92 | change current position without moveing head | X Y Z   | G1X10Y20F1000 |
| M | 3 | start spindle/laser | P X | M3P200 |
| M | 5 | stop spindle/laser | P X | M5 |
| M | 17 | start stepper motors | P X | M17 |
| M | 18 | stop stepper motors | P X | M18 |
