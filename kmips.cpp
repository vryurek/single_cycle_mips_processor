//
//  main.cpp
//  KMIPS
//
//  Created by Kit Cischke on 11/27/18.
//
// v0.1 Initial release. Supports only the minimum set of instructions.
// v0.1.1 Fixed the R-type instructions so that the opcode = 0 and the funct value in bits 5:0.
//        Also removed the boilerplate copyright notification that Xcode automatically adds.
// v0.1.2 Quick add of support for all-uppercase instruction mnemonics (e.g., BEQ).
// v0.1.3 Add in support for jump (j) and add immediate (addi) instructions. Also supports negative
//        offsets for loads and stores. Immediate values may be negative.
// v0.2.0 Adds support for addiu, andi, bne, ori and slti instructions. Tweaks the printout of
//        the machine code so that it always prints out 8 hex characters (pads with 0's).
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Silly license stuff:
// Copyright (c) 2018 Kit Cischke
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <iomanip>

using namespace std;

int main(int argc, const char * argv[])
{
    basic_string<char> currLineText;
    unsigned int currLineNum = 1;
    vector<string> tokens;
    stringstream currLineToTokenize;
    string intermediate;
    unsigned int currInstructionCode;

    // Print usage if not enough arguments
    if (argc != 2)
    {
        cout << "Usage: kmips input_file" << endl;
        cout << "Output always directed to a.out." << endl;
        exit(0);
    }

    // Print a friendly hello message
    cout << "Welcome to the simple MIPS assembler: KMIPS v0.2.0." << endl;

    // Check to see that the file exists.
    ifstream asmFile(argv[1]);
    if (!asmFile.is_open())
    {
        cerr << "Input file " << argv[1] << " not found. Exiting." << endl;
        exit(1);
    }

    // Open the output file
    ofstream outfile("a.out");

    // Read each line
    while (getline(asmFile, currLineText))
    {
        currInstructionCode = 0;

        //cout << currLineNum << ": " << currLineText << endl;
        outfile << "// " << currLineText << endl;

        currLineToTokenize << currLineText;

        // cout << "currLineToTokenize: " << currLineToTokenize.str() << endl;

        while (getline(currLineToTokenize, intermediate, ' '))
        {
            //cout << "Token: " << intermediate << endl;
            tokens.push_back(intermediate);
        }

        // Handle a comment line
        if (tokens.front() == "#")
        {
            //cout << "Found comment line. Ignoring the rest of the line." << endl;
            tokens.clear();
            // Clear the stringstream for the next line
            currLineToTokenize.str(string());
            currLineToTokenize.clear();
            // Update line number for next line
            currLineNum++;
            continue;
        }

        tokens.erase(tokens.begin());

        // Handle a blank line
        if (tokens.size() == 0)
        {
            tokens.clear();
            // Clear the stringstream for the next line
            currLineToTokenize.str(string());
            currLineToTokenize.clear();
            // Update line number for next line
            currLineNum++;
            continue;
        }
        // Interpret the opcode
        if ((tokens.front() == "lw") || (tokens.front() == "sw") ||
            (tokens.front() == "LW") || (tokens.front() == "SW"))
        {
            if (tokens.size() != 3)
            {
                cout << "Invalid number of operands for instruction on line " << currLineNum << ". Expected 3. Found " << tokens.size() << ". Exiting." << endl;
                exit(1);
            }

            // Here begins a lot of ridiculous code to isolate the register numbers (and offset)
            string rt(tokens[1]);
            rt.erase(0,1); // eliminate the $
            string::size_type comma = rt.find_first_of(",");
            rt.erase(comma, 1); // Kill the comma
            // cout << "rt: " << rt << endl;
            stringstream regPlusOffset(tokens[2]);
            int offset;
            regPlusOffset >> offset;
            string rs(tokens[2]);
            string::size_type cash = rs.find_first_of("$");
            rs.erase(0,cash+1);
            string::size_type closeParens = rs.find_first_of(")");
            rs.erase(closeParens, 1);

            int opcode = 0;
            if (tokens.front() == "lw")
            {
                opcode = 0x23;
            }
            else
            {
                opcode = 0x2B;
            }
            currInstructionCode |= (opcode << 26);
            currInstructionCode |= (atoi(rs.c_str()) << 21);
            currInstructionCode |= (atoi(rt.c_str()) << 16);
            currInstructionCode |= (offset & 0x0000FFFF);

            //cout << hex << currInstructionCode << endl;
            outfile << hex << setw(8) << setfill('0') << currInstructionCode << endl;

            tokens.clear();

        }
        else if ((tokens.front() == "addi") || (tokens.front() == "ADDI") ||
                 (tokens.front() == "addiu") || (tokens.front() == "ADDIU") ||
                 (tokens.front() == "andi") || (tokens.front() == "ANDI") ||
                 (tokens.front() == "ori") || (tokens.front() == "ORI") ||
                 (tokens.front() == "slti") || (tokens.front() == "SLTI"))
        {
            if (tokens.size() != 4)
            {
                cout << "Invalid number of operands for instruction on line " << currLineNum << ". Expected 4. Found " << tokens.size() << ". Exiting." << endl;
                exit(1);
            }
            
            // Here begins a lot of ridiculous code to isolate the register numbers (and offset)
            string rt(tokens[1]);
            rt.erase(0,1); // eliminate the $
            string::size_type comma = rt.find_first_of(",");
            rt.erase(comma, 1); // Kill the comma
            //cout << "rt: " << rt << endl;
            string rs(tokens[2]);
            rs.erase(0,1); // eliminate the $
            comma = rs.find_first_of(",");
            rs.erase(comma, 1);
            //cout << "rs: " << rs << endl;
            stringstream immediateVal(tokens[3]);
            int signExtImm;
            immediateVal >> signExtImm;
            //cout << "Immediate Value: " << signExtImm << endl;
            
            
            int opcode = 0;
            if ((tokens.front() == "addi") || (tokens.front() == "ADDI"))
            {
                opcode = 0x08;
            }
            else if ((tokens.front() == "addiu") || (tokens.front() == "ADDIU"))
            {
                opcode = 0x09;
            }
            else if ((tokens.front() == "andi") || (tokens.front() == "ANDI"))
            {
                opcode = 0x0C;
            }
            else if ((tokens.front() == "ori") || (tokens.front() == "ORI"))
            {
                opcode = 0x0D;
            }
            else if ((tokens.front() == "slti") || (tokens.front() == "SLTI"))
            {
                opcode = 0x0A;
            }
            currInstructionCode |= (opcode << 26);
            currInstructionCode |= (atoi(rs.c_str()) << 21);
            currInstructionCode |= (atoi(rt.c_str()) << 16);
            currInstructionCode |= (signExtImm & 0x0000FFFF);
            
            //cout << hex << currInstructionCode << endl;
            outfile << hex << setw(8) << setfill('0') << currInstructionCode << endl;
            
            tokens.clear();
            
        }
        else if ((tokens.front() == "and") || (tokens.front() == "AND") ||
                 (tokens.front() == "or")  || (tokens.front() == "OR")  ||
                 (tokens.front() == "add") || (tokens.front() == "ADD") ||
                 (tokens.front() == "sub") || (tokens.front() == "SUB") ||
                 (tokens.front() == "slt") || (tokens.front() == "SLT"))
        {
            if (tokens.size() != 4)
            {
                cout << "Invalid number of operands for instruction on line " << currLineNum << ". Expected 4. Found " << tokens.size() << ". Exiting." << endl;
                exit(1);
            }

            // Here begins a lot of ridiculous code to isolate the register numbers
            string rd(tokens[1]);
            rd.erase(0,1); // eliminate the $
            string::size_type comma = rd.find_first_of(",");
            rd.erase(comma, 1); // Kill the comma
            //cout << "rd: " << rd << endl;

            string rs(tokens[2]);
            rs.erase(0,1); // eliminate the $
            comma = rs.find_first_of(",");
            rs.erase(comma, 1); // Kill the comma
            //cout << "rs: " << rs << endl;

            string rt(tokens[3]);
            rt.erase(0,1); // eliminate the $
            //cout << "rt: " << rt << endl;

            int opcode = 0;
            int funct = 0;
            if ((tokens.front() == "and") || (tokens.front() == "AND"))
            {
                funct = 0x24;
            }
            else if ((tokens.front() == "or") || (tokens.front() == "OR"))
            {
                funct = 0x25;
            }
            else if ((tokens.front() == "add") || (tokens.front() == "ADD"))
            {
                funct = 0x20;
            }
            else if ((tokens.front() == "sub") || (tokens.front() == "SUB"))
            {
                funct = 0x22;
            }
            else
            {
                funct = 0x2A;
            }
            currInstructionCode |= (opcode << 26);
            currInstructionCode |= (atoi(rs.c_str()) << 21);
            currInstructionCode |= (atoi(rt.c_str()) << 16);
            currInstructionCode |= (atoi(rd.c_str()) << 11);
            currInstructionCode |= funct;
            
            //cout << "Machine code: " << hex << currInstructionCode << endl;
            outfile << hex << setw(8) << setfill('0') << currInstructionCode << endl;

            tokens.clear();
        }
        else if ((tokens.front() == "beq") || (tokens.front() == "BEQ")||
                 (tokens.front() == "bne") || (tokens.front() == "BNE"))
        {
            if (tokens.size() != 4)
            {
                cout << "Invalid number of operands for instruction on line " << currLineNum << ". Expected 4. Found " << tokens.size() << ". Exiting." << endl;
                exit(1);
            }

            // Here begins a lot of ridiculous code to isolate the register numbers and offset
            string rs(tokens[1]);
            rs.erase(0,1); // eliminate the $
            string::size_type comma;
            comma = rs.find_first_of(",");
            rs.erase(comma, 1); // Kill the comma
            //cout << "rs: " << rs << endl;

            string rt(tokens[2]);
            rt.erase(0,1); // eliminate the $
            comma = rt.find_first_of(",");
            rt.erase(comma, 1); // Kill the comma
            //cout << "rt: " << rt << endl;

            int offset = atoi(tokens[3].c_str());
            //cout << "offset: " << offset << endl;
            
            int opcode = 0;
            
            if ((tokens.front() == "beq") || (tokens.front() == "BEQ"))
            {
                opcode = 0x04;
            }
            else if ((tokens.front() == "bne") || (tokens.front() == "BNE"))
            {
                opcode = 0x05;
            }
            currInstructionCode |= (opcode << 26);
            currInstructionCode |= (atoi(rs.c_str()) << 21);
            currInstructionCode |= (atoi(rt.c_str()) << 16);
            currInstructionCode |= (offset & 0x0000FFFF);
            //cout << "Machine code: " << hex << currInstructionCode << endl;
            outfile << hex << setw(8) << setfill('0') << currInstructionCode << endl;
            tokens.clear();
        }
        else if (tokens.front() == "j" || tokens.front() == "J")
        {
            if (tokens.size() != 2)
            {
                cout << "Invalid number of operands for instruction on line " << currLineNum << ". Expected 2. Found " << tokens.size() << ". Exiting." << endl;
                exit(1);
            }
            
            // Here begins a lot of ridiculous code to isolate the register numbers and offset
            int address = atoi(tokens[1].c_str());
            //cout << "offset: " << offset << endl;
            
            currInstructionCode |= (0x02 << 26);
            currInstructionCode |= address;
            //cout << "Machine code: " << hex << currInstructionCode << endl;
            outfile << hex << setw(8) << setfill('0') << currInstructionCode << endl;
            tokens.clear();
        }
        else
        {
            cout << "Unrecognized instruction mnemonic on line " << currLineNum << ". Exiting." << endl;
            exit(1);
        }
        // Clear the stringstream for the next line
        currLineToTokenize.str(string());
        currLineToTokenize.clear();

        // Update line number for next line
        currLineNum++;
    }

    asmFile.close();
    outfile.close();

    return 0;
}
