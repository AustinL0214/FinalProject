%% Variables
N = 128;
saw_step = 4096/128;
a = 1:128;
saw_wave = ones(1,128);
i = 0;
x = 0;  
res    = 12;
offset = 0;
t = 0:((2*pi/(N-1))):(2*pi);

%% Sawtooth Wave
for c = a
    saw_wave(c) = c*saw_step;
end
saw_wave(128) = 4095;
% plot(a, saw_wave);
disp(saw_wave);
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
sine_wave = sin(t);
sine_wave = sine_wave + 1;
sine_wave = sine_wave*((2^res-1)-2*offset)/(2+offset); 
sine_wave = round(sine_wave);

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
        square_wave(c) = 4095;
    else
        square_wave(c) = 0;
    end
end

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
        triangle_wave(c) = (c)*64 - 1;
    else
        triangle_wave(c) = (128-c)*64;
    end
end

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