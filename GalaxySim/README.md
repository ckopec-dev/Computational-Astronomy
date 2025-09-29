# Galaxy simulation

~~~bash
# Run and output png files to the snapshots directory.
$ dotnet run

# Convert the images to an animated gif.
$ cd snapshots
$ sudo apt install imagemagick
$ convert -delay 5 -loop 0 snap_*.png galaxy.gif
~~~
