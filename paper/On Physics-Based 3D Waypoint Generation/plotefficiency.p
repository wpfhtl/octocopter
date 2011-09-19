set terminal png size 1600, 1200 font "times" 12
set output "graph_points_vs_time.png"

#set terminal postscript eps  font "times" 18
#set output "graph_points_vs_time.eps"

set autoscale

set xlabel "Elapsed Time (s)"
set ylabel "Pointcloud Size (points)"

set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically

#set grid

set key left top
#set key right bottom

set style line 1 lt 2 lc rgb "red" lw 3
set style line 2 lt 2 lc rgb "orange" lw 3
set style line 3 lt 2 lc rgb "cyan" lw 3
set style line 4 lt 2 lc rgb "green" lw 3
set style line 5 lt 2 lc rgb "blue" lw 3
set style line 6 lt 2 lc rgb "steelblue" lw 3
set style line 7 lt 2 lc rgb "grey" lw 3
set style line 8 lt 2 lc rgb "yellow" lw 3
set style line 9 lt 2 lc rgb "black" lw 3
set style line 10 lt 2 lc rgb "#4682b4" lw 3

# 3 segments means 4 passes

set arrow 1 from 8,0 to 8,4000 nohead lw 1 lc rgb "red"
set arrow 2 from 105,0 to 105,94500 nohead lw 1 lc rgb "red"
set arrow 3 from 231,0 to 231,232500 nohead lw 1 lc rgb "red"
set arrow 4 from 359,0 to 359,320000 nohead lw 1 lc rgb "red"

#plot "stats-lines-04.txt" using 1:2 ls 2 title "4 straight passes" with lines,\
#     "stats-lines-05.txt" using 1:2 ls 3 title "5 straight passes" with lines,\
#     "stats-lines-06.txt" using 1:2 ls 4 title "6 straight passes" with lines,\
#     "stats-lines-07.txt" using 1:2 ls 5 title "7 straight passes" with lines,\
#     "stats-lines-08.txt" using 1:2 ls 6 title "8 straight passes" with lines,\
#     "stats-physics-6m-2m-new.txt" using 1:2 ls 1 title "Our algorithm" with lines

plot "stats-lines-04-new.txt" using 1:2 ls 2 title "4 straight passes" with lines,\
     "stats-lines-05-new.txt" using 1:2 ls 3 title "5 straight passes" with lines,\
     "stats-lines-06-new.txt" using 1:2 ls 4 title "6 straight passes" with lines,\
     "stats-lines-07-new.txt" using 1:2 ls 5 title "7 straight passes" with lines,\
     "stats-lines-08-new.txt" using 1:2 ls 6 title "8 straight passes" with lines,\
     "stats-physics-6m-2m-new.txt" using 1:2 ls 1 title "Our algorithm" with lines,\
     "stats-physics-new.txt" using 1:2 ls 7 title "Our algorithm" with lines