classdef Biquad < handle
    
    properties(SetAccess = private)
        a0 = 1.0;
        a1 = 0.0;
        a2 = 0.0;
        b0 = 1.0;
        b1 = 0.0;
        b2 = 0.0;
        fn = 10.0;
        Q  = 0.707;
        z1 = 0.0;
        z2 = 0.0;
        peakGain = 0.0;
        TYPE; 
    end
    
   methods
       function obj = Biquad( arType, arFn, arQ, arGain)
            obj.TYPE = arType;
            obj.fn   = arFn;
            obj.Q    = arQ;
            obj.peakGain = arGain;
            obj.CalcBiquad();
        end
        
        function outData = Filter(obj, arData)
            inVal  = arData;
            outVal = inVal*obj.a0 + obj.z1;
            obj.z1 = inVal*obj.a1 + obj.z2 - obj.b1*outVal;
            obj.z2 = inVal*obj.a2 - obj.b2*outVal;
            outData = outVal;
        end
        
        function  CalcBiquad(obj)
            k = tan(pi*obj.fn);
            v = 10.0^(abs(obj.peakGain)/20.0);
            switch  obj.TYPE
                case 'LOWPASS'   %lowpass
                    norm = 1.0/(1 + k/obj.Q + k*k);
                    obj.a0 = k*k*norm;
                    obj.a1 = 2*obj.a0;
                    obj.a2 = obj.a0;
                    obj.b1 = 2*(k*k - 1)*norm;
                    obj.b2 = ((1.0 - k/obj.Q + k*k)*norm);
                case 'HIGHPASS'   %highpass
                    norm = 1.0/(1 + k/obj.Q + k*k);
                    obj.a0 = 1.0*norm;
                    obj.a1 = -2.0*obj.a0;
                    obj.a2 = obj.a0;
                    obj.b1 = 2*(k*k - 1)*norm;
                    obj.b2 = (1.0 - k/obj.Q + k*k)*norm;
                case 'BANDPASS'	 %bandpass
                    norm = 1.0/(1 + k/obj.Q + k*k);
                    obj.a0 = k/obj.Q*norm;
                    obj.a1 = 0;
                    obj.a2 = -obj.a0;
                    obj.b1 = 2*(k*k - 1)*norm;
                    obj.b2 = ((1.0 - k/obj.Q + k*k)*norm);
                case 'NOTCH'   %notch
                    norm = 1.0/(1 + k/obj.Q + k*k);
                    obj.a0 = (1.0 + k*k)*norm;
                    obj.a1 = 2*(k*k - 1.0)*norm;
                    obj.a2 = obj.a0;
                    obj.b1 = obj.a1;
                    obj.b2 = (1.0 - k/obj.Q + k*k)*norm;
                    
                case 'PEAK'  %peak
                    if (obj.peakGain >= 0)
                        % boost
                        norm = 1 / (1 + 1/obj.Q * k + k * k);
                        obj.a0 = (1 + v/obj.Q * k + k * k) * norm;
                        obj.a1 = 2 * (k * k - 1) * norm;
                        obj.a2 = (1 - v/obj.Q * k + k * k) * norm;
                        obj.b1 = obj.a1;
                        obj.b2 = (1 - 1/obj.Q * k + k * k) * norm;
                    else
                        % cut
                        norm = 1 / (1 + v/obj.Q * k + k * k);
                        obj.a0 = (1 + 1/obj.Q * k + k * k) * norm;
                        obj.a1 = 2 * (k * k - 1) * norm;
                        obj.a2 = (1 - 1/obj.Q * k + k * k) * norm;
                        obj.b1 = obj.a1;
                        obj.b2 = (1 - v/obj.Q * k + k * k) * norm;
                    end
                    
                case 'LOWSHELF'  %lowshelf filter
                    if (obj.peakGain >= 0)
                        % boost
                        norm = 1 / (1 + 2^0.5 * k + k * k);
                        obj.a0 = (1 + (2*v)^0.5 * k + v * k * k) * norm;
                        obj.a1 = 2 * (v * k * k - 1) * norm;
                        obj.a2 = (1 - (2*v)^0.5 * k + v * k * k) * norm;
                        obj.b1 = 2 * (k * k - 1) * norm;
                        obj.b2 = (1 - 2^0.5 * k + k * k) * norm;
                    else
                        % cut
                        norm = 1 / (1 + (2*v)^0.5 * k + v * k * k);
                        obj.a0 = (1 + 2^0.5 * k + k * k) * norm;
                        obj.a1 = 2 * (k * k - 1) * norm;
                        obj.a2 = (1 - 2^0.5 * k + k * k) * norm;
                        obj.b1 = 2 * (v * k * k - 1) * norm;
                        obj.b2 = (1 - (2*v)^0.5 * k + v * k * k) * norm;
                    end
                case 'HIGHSHELF'  %highshelf filter
                    if (obj.peakGain >= 0)
                        % boost
                        norm = 1 / (1 + 2^0.5 * k + k * k);
                        obj.a0 = (v + (2*v)^0.5 * k + k * k) * norm;
                        obj.a1 = 2 * (k * k - v) * norm;
                        obj.a2 = (v - (2*v)^0.5 * k + k * k) * norm;
                        obj.b1 = 2 * (k * k - 1) * norm;
                        obj.b2 = (1 - 2^0.5 * k + k * k) * norm;
                    else
                        % cut
                        norm = 1 / (v + (2*v)^0.5 * k + k * k);
                        obj.a0 = (1 + 2^0.5 * k + k * k) * norm;
                        obj.a1 = 2 * (k * k - 1) * norm;
                        obj.a2 = (1 - 2^0.5 * k + k * k) * norm;
                        obj.b1 = 2 * (k * k - v) * norm;
                        obj.b2 = (v - (2*v)^0.5 * k + k * k) * norm;
                    end
                case 'MATLABLOW'
%                     [b, a] = ellip(2, obj.Q, 80, 2*obj.fn, 'low');
%                     obj.a0 = b(1);
%                     obj.a1 = b(2);
%                     obj.a2 = b(3);
%                     obj.b1 = a(2);
%                     obj.b2 = a(3);
                    [b, a] = butter(2, 2*obj.fn, 'low');
                    obj.a0 = b(1);
                    obj.a1 = b(2);
                    obj.a2 = b(3);
                    obj.b1 = a(2);
                    obj.b2 = a(3);
                otherwise
                    error('Invalid type');
            end
        end
        
   end 
   
end
