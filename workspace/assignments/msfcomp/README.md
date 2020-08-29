# Solution to msfcomp

This is the solution to `msfcomp`

---

## Up and Running

Use the following commands to build the solution

```bash
# build solution:
mkdir build && cd build && cmake .. && make -j2
# run on test data:
./msfcomp --timestamp 123 --ignore_key id ../data/A.txt ../data/B.txt
```

The sample output for Q1 is shown below:

<img src="doc/sample-output.png" alt="Sample Output" width="%100">