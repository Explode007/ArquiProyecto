module ConditionHandler(
    input B_in,
    input BL_in,
    input [3:0] I_Cond_in,

    input Z_in,
    input N_in,
    input C_in,
    input V_in,

    output reg TA_Ctrl_out,
    output reg BL_COND_out,
    output reg COND_EVAL_out
    );

    parameter EQUALS            = 4'b0000;
    parameter NOT_EQUALS        = 4'b0001;
    parameter CARRY_SET         = 4'b0010;
    parameter CARRY_CLEAR       = 4'b0011;
    parameter MINUS             = 4'b0100;
    parameter PLUS              = 4'b0101;
    parameter OVERFLOW          = 4'b0110;
    parameter NO_OVERFLOW       = 4'b0111;
    parameter UNSIGNED_HIGHER   = 4'b1000;
    parameter UNSIGNED_LOWER    = 4'b1001;
    parameter GREATER_EQUAL     = 4'b1010;
    parameter LESS_THAN         = 4'b1011;
    parameter GREATER_THAN      = 4'b1100;
    parameter LESS_EQUAL        = 4'b1101;
    parameter ALWAYS            = 4'b1110;
    parameter NEVER             = 4'b1111;

    always @* begin
        TA_Ctrl_out = 0;
        BL_COND_out = 0;
        COND_EVAL_out = 0;

        case (I_Cond_in)
            EQUALS:           COND_EVAL_out = Z_in;
            NOT_EQUALS:       COND_EVAL_out = ~Z_in;
            CARRY_SET:        COND_EVAL_out = C_in;
            CARRY_CLEAR:      COND_EVAL_out = ~C_in;
            MINUS:            COND_EVAL_out = N_in;
            PLUS:             COND_EVAL_out = ~N_in;
            OVERFLOW:         COND_EVAL_out = V_in;
            NO_OVERFLOW:      COND_EVAL_out = ~V_in;
            UNSIGNED_HIGHER:  COND_EVAL_out = C_in && ~Z_in;
            UNSIGNED_LOWER:   COND_EVAL_out = ~C_in || Z_in;
            GREATER_EQUAL:    COND_EVAL_out = (N_in == V_in);
            LESS_THAN:        COND_EVAL_out = (N_in != V_in);
            GREATER_THAN:     COND_EVAL_out = ~Z_in && (N_in == V_in);
            LESS_EQUAL:       COND_EVAL_out = Z_in || (N_in != V_in);
            ALWAYS:           COND_EVAL_out = 1;
            NEVER:            COND_EVAL_out = 0;
            default:          COND_EVAL_out = 0;
        endcase

        TA_Ctrl_out = (B_in || BL_in) && COND_EVAL_out;
        BL_COND_out = BL_in && COND_EVAL_out;
    end

endmodule