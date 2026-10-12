// vlsu_pkg.sv
// Instruction type encoding for the 8-lane Vector LSU.
// Covers all six RVV-style unit-stride / strided / indexed load+store variants.

package vlsu_pkg;

  typedef enum logic [2:0] {
    // Load instructions
    VLE  = 3'd0,   // Vector Load  – unit-stride          (vle{8,16,32,64}.v)
    VLSE = 3'd1,   // Vector Load  – strided               (vlse{...}.v)
    VLXE = 3'd2,   // Vector Load  – indexed               (vlxe{...}.v / vluxe / vloxe)
    // Store instructions
    VSE  = 3'd4,   // Vector Store – unit-stride          (vse{8,16,32,64}.v)
    VSSE = 3'd5,   // Vector Store – strided               (vsse{...}.v)
    VSXE = 3'd6    // Vector Store – indexed               (vsxe{...}.v / vsuxe / vsoxe)
  } vlsu_insn_e;

  // Convenience decode functions callable in RTL
  function automatic logic insn_is_store (input vlsu_insn_e insn);
    return insn[2];   // bit[2]==1 for VSE/VSSE/VSXE
  endfunction

  function automatic logic insn_is_strided (input vlsu_insn_e insn);
    return (insn == VLSE) | (insn == VSSE);
  endfunction

  function automatic logic insn_is_indexed (input vlsu_insn_e insn);
    return (insn == VLXE) | (insn == VSXE);
  endfunction

endpackage
