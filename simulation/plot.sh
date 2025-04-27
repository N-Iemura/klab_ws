#!/bin/bash

gnuplot -persist <<-EOFMarker
    set terminal pngcairo size 800,600
    set output 'plot.png'
    set multiplot layout 3, 1 title "Simulation Results"

    set title "Position Response"
    plot 'data.txt' using 1 with lines title 'Position'

    set title "Velocity Response"
    plot 'data.txt' using 2 with lines title 'Velocity'

    set title "Force Response"
    plot 'data.txt' using 3 with lines title 'Force'

    unset multiplot
EOFMarker