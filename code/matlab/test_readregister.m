%**************************************************************************
% Script Name    : test_readregister
% Description    : 
%  This script runs through each of the given registers and prints out the
%  result of a read register request.  Note what happens on register 41
%  which is not a valid register.  When this occurs an error code is
%  returned and printed to the screen. In the case of an error a zero
%  length array is returned.
%**************************************************************************
clc;

s = VNserial('COM8');

reg = [1 2 3 4 5 6 7 8 9 15 17 18 19 20 21 23 26 27 30 32 54];

for i = 1:length(reg)
   fprintf('-----  Reg %u ---------------------\n', reg(i));
   data = VNreadregister(s,reg(i));
   disp(data);
   fprintf('\n');
end
