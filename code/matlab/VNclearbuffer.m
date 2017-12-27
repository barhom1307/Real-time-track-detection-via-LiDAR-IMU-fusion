%**************************************************************************
% Function Name  : VNclearbuffer
% Description    : 
%  VNclearbuffer clears the input buffer for the given serial port. This
%   function can be used to clear the input buffer after you turn off the
%   streaming data output by setting ADOR = 0.  After clearing the input
%   buffer you can now issue commands and look for the response from the
%   given command without accidently receiving responses from the streaming
%   ADOR data.
%
% Input(s)       : VNserial -> serial port object
% Output(s)      : None
%**************************************************************************
%   Examples:
%
%   Turn the Asynchronous Data Output Register (ADOR) off and then clear the
%   input buffer.
%
%   s = VNserial('COM8');
%   
%   VNwriteregister(s, 06, 0);
%   VNclearbuffer(s);
%
%   Change Log: 12/20/2010 - Removed dependency for the instrument toolbox
%                            for the flushinput command.
%
% ------------- VectorNav Technologies, LLC -------------------------------
% This file is property of VectorNav Technologies and cannot be used,
% copied or distributed without the written consent of VectorNav
% Technologies. 
% -------------------------------------------------------------------------
function VNclearbuffer(VNserial)

    %Clear input buffer
    while(VNserial.BytesAvailable > 0)
        %Old method of flushing the input buffer. This requires the
        %instrument toolbox.
        %flushinput(VNserial);
        
        %New method
        if(VNserial.BytesAvailable > 0)
            fread(VNserial, VNserial.BytesAvailable, 'uint8');
        end
        
        pause(0.1);
    end
