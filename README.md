# Multi-Cycle 16-bit RISC Processor (Verilog)

A multi-cycle RISC processor designed and implemented in Verilog as part of a Computer Architecture project.  
The processor executes a custom 16-bit Instruction Set Architecture (ISA) with support for arithmetic, logic, branching, memory access, and function call operations.

---

## Processor Architecture Overview

### Key Design Characteristics
- **Multi-Cycle Execution Architecture**
  - Efficient hardware utilization with state-based instruction progression
- **16-bit word and instruction size**
- **8 General-Purpose Registers** (`R0`–`R7`)
  - `R0` hardwired to zero (write ignored)
- **Program Counter (PC): 16-bit**
- **Harvard Architecture**  
  - Separate Instruction and Data memories
- **Byte-addressable memory**
- **Little-endian data ordering**

### Supported Instruction Types
| Type | Description |
|------|-------------|
| R-Type | Register-to-register ALU operations |
| I-Type | Immediate arithmetic, load/store, conditional branches |
| J-Type | Unconditional jump + function call |
| S-Type | Store immediate |

A total of **21 instructions** implemented, including:  
`ADD, SUB, AND, ADDI, ANDI, LW, LBu, LBs, SW, BGT, BGTZ, BLT, BLTZ, BEQ, BEQZ, BNE, BNEZ, JMP, CALL, RET, Sv`

---

## Datapath & Control

The processor executes each instruction over multiple cycles:  
**Fetch → Decode → Execute → Memory → Write-Back**

### Control Unit
- Finite-State Machine (FSM)
- Generates all control signals per instruction phase
- Supports immediate extension and branch decision evaluation logic

### ALU Features
- Zero, carry, and overflow detection
- Supports signed/unsigned arithmetic as required by instructions

---

## Verification & Testing

- **Comprehensive Verilog Testbench**
- Instruction memory preloaded with custom test programs
- Validation through:
  - Simulations
  - Waveform inspection
- All ISA instructions tested for expected functional behavior

---

## Contributors
- **Sondos Qasarwa**  
- **Sarah Hassouneh**
