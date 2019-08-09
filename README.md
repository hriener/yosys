This repository is a clone of yosys (Yosys Open SYnthesis Suite) extended with an experimental command `cirkit` that allows integrating logic optimisations of `mockturtle` in RTL synthesis.

**Disclaimer**: The project is still under development and only conceptually demonstrates how the integration of `yosys` and `mockturtle` could work.  The `cirkit` command is heavily based on the `abc` command and makes only a few minor modifications to allow its users to run CirKit scripts in `yosys`. Consequently, the `cirkit` command inherets serveral arguments from the `abc` command which are not support.  Invoking the `cirkit` command with these arguments will lead to failure, incorrect results, or other undefined behavior.

For further information, see
* mockturtle: C++ logic network library, https://github.com/lsils/mockturtle
* CirKit: a circuit toolkit (serves as frontend for `mockturtle`), https://github.com/msoeken/cirkit
* yosys: a framework for RTL synthesis, http://www.clifford.at/yosys/

Installation
------------
- First, `yosys` has to be build in the usual way
- Then, `cirkit` has to be build
- Finally, the `cirkit` executable needs to be copied to the `yosys` directory and renamed to `yosys-cirkit`

Usage
-----
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
