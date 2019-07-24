import numpy as numpy

class FirmwareGenerator:
    def __init__(self, W0, b0, W_hidden, b_hidden, W1, b1, \
        num_hidden_layer, ob_space_size, hidden_layer_size, ac_space_size, \
        state_size, final_bias, ob_mean, ob_std):
        self.W0 = W0
        self.b0 = b0
        self.W_hidden = W_hidden
        self.b_hidden = b_hidden
        self.W1 = W1
        self.b1 = b1
        self.num_hidden_layer = num_hidden_layer
        self.ob_space_size = ob_space_size
        self.hidden_layer_size = hidden_layer_size
        self.ac_space_size = ac_space_size
        self.state_size = state_size
        self.final_bias = final_bias
        self.ob_mean = ob_mean
        self.ob_std = ob_std

    def generate(self):
        filename = "AP_MotorsPolicyDefinition.h"

        fp = open(filename, "w")
        fp.write("#ifndef __AP_MOTORS_POLICY_DEF_H__\n")
        fp.write("#define __AP_MOTORS_POLICY_DEF_H__\n")

        fp.write("\n")
        fp.write("const int STATE_SIZE = %i;\n"%self.state_size)
        fp.write("const int OB_SPACE_SIZE = %i;\n"%self.ob_space_size)
        fp.write("const int AC_SPACE_SIZE = %i;\n"%self.ac_space_size)
        fp.write("const int NUM_HIDDEN_LAYER = %i;\n"%self.num_hidden_layer)
        fp.write("const int HIDDEN_LAYER_SIZE = %i;\n"%self.hidden_layer_size)
        fp.write("const float FINAL_BIAS = %f;\n"%self.final_bias)

        fp.write("\n")
        fp.write("const float W0[OB_SPACE_SIZE][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.ob_space_size):
            fp.write("{")
            for j in range(self.hidden_layer_size):
                fp.write("%f, "%self.W0[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float W_hidden[NUM_HIDDEN_LAYER - 1][HIDDEN_LAYER_SIZE][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.num_hidden_layer - 1):
            fp.write("{\n")
            for j in range(self.hidden_layer_size):
                fp.write("{")
                for k in range(self.hidden_layer_size):
                    fp.write("%f, "%self.W_hidden[i][j][k])
                fp.write("},\n")
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float W1[HIDDEN_LAYER_SIZE][AC_SPACE_SIZE] = {\n")
        for i in range(self.hidden_layer_size):
            fp.write("{")
            for j in range(self.ac_space_size):
                fp.write("%f, "%self.W1[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b0[HIDDEN_LAYER_SIZE] = {")
        for i in range(self.hidden_layer_size):
            fp.write("%f, "%self.b0[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b_hidden[NUM_HIDDEN_LAYER - 1][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.num_hidden_layer - 1):
            fp.write("{")
            for j in range(self.hidden_layer_size):
                fp.write("%f, "%self.b_hidden[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b1[AC_SPACE_SIZE] = {")
        for i in range(self.ac_space_size):
            fp.write("%f, "%self.b1[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float ob_mean[OB_SPACE_SIZE] = {")
        for i in range(self.ob_space_size):
            fp.write("%f, "%self.ob_mean[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float ob_std[OB_SPACE_SIZE] = {")
        for i in range(self.ob_space_size):
            fp.write("%f, "%self.ob_std[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("#endif\n")

        fp.close()

        filename = "AP_MotorsPolicyDefinition_UI.h"

        fp = open(filename, "w")
        fp.write("#ifndef __AP_MOTORS_POLICY_DEF_H__\n")
        fp.write("#define __AP_MOTORS_POLICY_DEF_H__\n")

        fp.write("\n")

        fp.write("namespace copter_simulation {\n")
        fp.write("\n")

        fp.write("const int STATE_SIZE = %i;\n"%self.state_size)
        fp.write("const int OB_SPACE_SIZE = %i;\n"%self.ob_space_size)
        fp.write("const int AC_SPACE_SIZE = %i;\n"%self.ac_space_size)
        fp.write("const int NUM_HIDDEN_LAYER = %i;\n"%self.num_hidden_layer)
        fp.write("const int HIDDEN_LAYER_SIZE = %i;\n"%self.hidden_layer_size)
        fp.write("const float FINAL_BIAS = %f;\n"%self.final_bias)

        fp.write("\n")
        fp.write("const float W0[OB_SPACE_SIZE][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.ob_space_size):
            fp.write("{")
            for j in range(self.hidden_layer_size):
                fp.write("%f, "%self.W0[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float W_hidden[NUM_HIDDEN_LAYER - 1][HIDDEN_LAYER_SIZE][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.num_hidden_layer - 1):
            fp.write("{\n")
            for j in range(self.hidden_layer_size):
                fp.write("{")
                for k in range(self.hidden_layer_size):
                    fp.write("%f, "%self.W_hidden[i][j][k])
                fp.write("},\n")
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float W1[HIDDEN_LAYER_SIZE][AC_SPACE_SIZE] = {\n")
        for i in range(self.hidden_layer_size):
            fp.write("{")
            for j in range(self.ac_space_size):
                fp.write("%f, "%self.W1[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b0[HIDDEN_LAYER_SIZE] = {")
        for i in range(self.hidden_layer_size):
            fp.write("%f, "%self.b0[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b_hidden[NUM_HIDDEN_LAYER - 1][HIDDEN_LAYER_SIZE] = {\n")
        for i in range(self.num_hidden_layer - 1):
            fp.write("{")
            for j in range(self.hidden_layer_size):
                fp.write("%f, "%self.b_hidden[i][j])
            fp.write("},\n")
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float b1[AC_SPACE_SIZE] = {")
        for i in range(self.ac_space_size):
            fp.write("%f, "%self.b1[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float ob_mean[OB_SPACE_SIZE] = {")
        for i in range(self.ob_space_size):
            fp.write("%f, "%self.ob_mean[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("const float ob_std[OB_SPACE_SIZE] = {")
        for i in range(self.ob_space_size):
            fp.write("%f, "%self.ob_std[i])
        fp.write("};\n")

        fp.write("\n")
        fp.write("}\n")
        
        fp.write("#endif\n")

        fp.close()

