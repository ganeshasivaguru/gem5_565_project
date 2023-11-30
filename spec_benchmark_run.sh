
mkdir m5out/split_ex_dup/

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b sjeng --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/sjeng.txt

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b leslie3d --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/leslie3d.txt

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b lbm --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/lbm.txt

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b astar --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/astar.txt

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b milc --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/milc.txt

./build/ECE565-ARM/gem5.opt configs/spec/spec_se.py --cpu-type=MinorCPU -b namd --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache
mv m5out/stats.txt m5out/split_ex_dup/namd.txt

./build/ECE565-ARM/gem5.opt configs/example/se.py --cpu-type=MinorCPU --maxinsts=1000000 --l1d_size=64kB --l1i_size=16kB --caches --l2cache -c part1/daxpy-armv7-binary
mv m5out/stats.txt m5out/split_ex_dup/daxpy.txt
