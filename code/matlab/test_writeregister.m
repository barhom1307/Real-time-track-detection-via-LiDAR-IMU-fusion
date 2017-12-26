%**************************************************************************
% Script Name    : test_writeregister
% Description    : 
%  This script runs through each of the given registers and prints out the
%  result of a read & write register request.  Note what happens on 
%  register 41 which is not a valid register.  When this occurs an error
%  code is returned and printed to the screen. In the case of an error a
%  zero length array is returned.
%**************************************************************************
clc;

s = VNserial('COM8');

reg = [5 6 7 21 23 26 30 32];

for i = 1:length(reg)
  fprintf('-----  Reg %u ---------------------\n', reg(i));
  data = VNreadregister(s,reg(i));
  if ~isempty(data)
    fprintf('Read : ');
    disp(data);
  end
    data = VNwriteregister(s,reg(i),data);
    if ~isempty(data)
      fprintf('Write: ');
      disp(data);
    end
    fprintf('\n');
end
