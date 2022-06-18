%% precalc
x = 0.05*sin(0:0.1:2*pi);
y = 0.35+0.05*cos(0:0.1:2*pi);

%% CV
I = imread('input.jpg');
BW = im2bw(I);
BW = ~BW;
[B,L,N] super().moveJ(point, blocked)= bwboundaries(BW);

%% Serial works
clear s;
s = serialport("/dev/cu.usbserial-1410", 115200, "Timeout", 30);

while(s.NumBytesAvailable < 1) end
read(s, s.NumBytesAvailable, "string")

for k = 1:length(B)
    boundary = B{k};
    write(s, [ik(0.0001*boundary(1,2) - 0.05, 0.0001*boundary(1,1) + 0.3) 0], "single");
    read(s, 2, "string");
    write(s, [0 0 1], "single"); %Move down
    read(s, 2, "string")
    
    for i= 1:size(boundary, 1)
        write(s, [ik(0.0001*boundary(i,2) - 0.05, 0.0001*boundary(i,1) + 0.3) 0], "single");
        read(s, 2, "string");
    end
    
    write(s, [0 0 2], "single"); %Move up
    read(s, 2, "string")
end

clear s;