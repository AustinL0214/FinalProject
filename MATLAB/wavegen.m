%% Variables
N = 128;    %Sample size
saw_step = 4096/128;    %Step size for sawtooth wave
a = 1:128;  % buffer to hold lookup table values
saw_wave = ones(1,128);
i = 0;
x = 0;  
res    = 12;
offset = 0;
t = 0:((2*pi/(N-1))):(2*pi);    % time for sine wave

%% Sawtooth Wave
for c = a
    saw_wave(c) = c*saw_step;   % 0 to 4095 
end
saw_wave(128) = 4095;   % last value 4095
% plot(a, saw_wave);
%disp(saw_wave);     
% output to C code
fprintf("uint32_t Saw_LUT[NS] = {\n");
fprintf("\t");
for d = a
    if (x == 127)
        fprintf("%d", saw_wave(d));
    else
        fprintf("%d, ", saw_wave(d));
    end
    i = i + 1;
    x = x + 1;
    if (i == 16)
        i = 0;
        fprintf("\n\t");
    end
end
fprintf("};\n");

%% Sine Wave
sine_wave = sin(t); %create sine wave values
sine_wave = sine_wave + 1;
sine_wave = sine_wave*((2^res-1)-2*offset)/(2+offset); 
sine_wave = round(sine_wave);
% Output to C code
fprintf("uint32_t Sine_LUT[NS] = {\n");
fprintf("\t");
for d = a
    if (x == 127)
        fprintf("%d", sine_wave(d));
    else
        fprintf("%d, ", sine_wave(d));
    end
    i = i + 1;
    x = x + 1;
    if (i == 16)
        i = 0;
        fprintf("\n\t");
    end
end
fprintf("};\n");

%% Square Wave
square_wave = 1:128;
for c = a
    if(c < 65)
        square_wave(c) = 4095;  % either 4095 or 0
    else
        square_wave(c) = 0;
    end
end
% Output to C code
fprintf("uint32_t Square_LUT[NS] = {\n");
fprintf("\t");
for d = a
    if (x == 127)
        fprintf("%d", square_wave(d));
    else
        fprintf("%d, ", square_wave(d));
    end
    i = i + 1;
    x = x + 1;
    if (i == 16)
        i = 0;
        fprintf("\n\t");
    end
end
fprintf("};\n");

%% Triangle Wave
triangle_wave = 1:128;
for c = a
    if (c < 65)
        triangle_wave(c) = (c)*64 - 1;  % increasing to 4095
    else
        triangle_wave(c) = (128-c)*64;  % decreasing to 0
    end
end
% Output to C code
fprintf("uint32_t Triangle_LUT[NS] = {\n");
fprintf("\t");
for d = a
    if (x == 127)
        fprintf("%d", triangle_wave(d));
    else
        fprintf("%d, ", triangle_wave(d));
    end
    i = i + 1;
    x = x + 1;
    if (i == 16)
        i = 0;
        fprintf("\n\t");
    end
end
fprintf("};\n");

%% Zero voltage
zero_voltage = zeros(1, 128);

% Output to C code
fprintf("uint32_t Zero_LUT[NS] = {\n");
fprintf("\t");
for d = a
    if (x == 127)
        fprintf("%d", zero_voltage(d));
    else
        fprintf("%d, ", zero_voltage(d));
    end
    i = i + 1;
    x = x + 1;
    if (i == 16)
        i = 0;
        fprintf("\n\t");
    end
end
fprintf("};\n");