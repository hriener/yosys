This repository is a clone of yosys (Yosys Open SYnthesis Suite) extended with an experimental command `cirkit` that allows integrating logic optimisations of `mockturtle` in RTL synthesis.

**Disclaimer**: The project is still under development and only conceptually demonstrates how the integration of `yosys` and `mockturtle` could work.  The `cirkit` command is heavily based on the `abc` command and makes only a few minor modifications to allow its users to run CirKit scripts in `yosys`. Consequently, the `cirkit` command inherets serveral options and flags from the `abc` command which are not support.  Invoking the `cirkit` command with these options and flags will lead to failure, incorrect results, or other undefined behavior.

For further information, see
* mockturtle: C++ logic network library, https://github.com/lsils/mockturtle
* CirKit: a circuit toolkit (serves as frontend for `mockturtle`), https://github.com/msoeken/cirkit
* yosys: a framework for RTL synthesis, http://www.clifford.at/yosys/

Installation
============
- First, `yosys` has to be build in the usual way
- Then, `cirkit` has to be build
- Finally, the `cirkit` executable needs to be copied to the `yosys` directory and renamed to `yosys-cirkit`

Usage
=====

Execute CirKit scripts
----------------------
```bash
$ cat example.v
```
```verilog
module top (input clk, input [7:0] a, b, output reg [15:0] c);
  always @(posedge clk) c <= a * b;
endmodule // top
```

```bash
$ cat opt.cs
```
```bash
ps -l
lut_resynthesis --strategy=0
ps -m
cut_rewrite -m --strategy=0
```

```bash
$ ./yosys -p "prep; techmap; cirkit -script opt.cs; write_verilog example_yosys.v" example.v
```
```bash
[...]
CIRKIT: LUT network   i/o = 16/16   gates = 630   level = 28
CIRKIT: MIG   i/o = 16/16   gates = 680   level = 54
CIRKIT: MIG   i/o = 16/16   gates = 490   level = 47
[...]
CIRKIT RESULTS:              $lut cells:      506
CIRKIT RESULTS:        internal signals:      739
CIRKIT RESULTS:           input signals:       16
CIRKIT RESULTS:          output signals:       16
[...]
```

Verify the transformations executed by the script
-------------------------------------------------
```bash
$ ./yosys -p "prep; techmap; cirkit -nocleanup -showtmp -script opt.cs; write_verilog example_yosys.v" example.v
$ ./abc/cec -c "cec _tmp_yosys-cirkit-*/input.blif _tmp_yosys-cirkit-*/output.blif"
```
```bash
ABC command line: "cec -n _tmp_yosys-cirkit-*/input.blif _tmp_yosys-cirkit-*/output.blif".

Networks are equivalent.  Time =     0.15 sec
```

End-to-end equivalence checking
-------------------------------
```bash
$ ./yosys
read_verilog example.v; prep -flatten -top top; splitnets -ports; design -stash gold
read_verilog example_yosys.v; prep -flatten -top top; splitnets -ports; design -stash gate
design -copy-from gold -as gold top
design -copy-from gate -as gate top
equiv_make gold gate equiv
prep -flatten -top equiv
opt_clean -purge
opt -full
equiv_simple -seq 5
equiv_induct -seq 5
equiv_status -assert
```

```bash
11. Executing EQUIV_SIMPLE pass.
Found 16 unproven $equiv cells (16 groups) in equiv:
[...]
Proved 0 previously unproven $equiv cells.

12. Executing EQUIV_INDUCT pass.
Found 16 unproven $equiv cells in module equiv:
[...]
Proved 16 previously unproven $equiv cells.

Found 16 $equiv cells in equiv:
  Of those cells 16 are proven and 0 are unproven.
  Equivalence successfully proven!
```
