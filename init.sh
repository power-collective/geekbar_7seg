glasgow safe
glasgow run probe-rs -V 5 --swclk A1 --swdio A0
probe-rs gdb --probe 20b7:9db1:C3-20240303T140559Z:1:0 --protocol=swd --speed=1000 --chip=py32f003x6 --reset-halt --gdb-connection-string 127.0.0.1:1337
