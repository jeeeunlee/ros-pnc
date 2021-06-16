clc; clear all; 
abc = [  1.649547   1.650865   4.654343  
 504.376642   235.093313   1206.336952  
-0.051791   494.120397   1262.101554  
 832.858359  -124.996143   63.067026  
-1342.002019  -1.995310  -2482.461558  
-497.679540  -231.971911  -1002.149766  
 0.053014  -508.499232  -939.523111  
-555.712113   83.401154   409.210374  
 308.130543   0.457858   787.447036  
-1.582308  -1.583573  -2.506052  
 2.091693  -0.144301   2.332277  
-2.180579   0.150433  -2.038432  
 0.000002   0.000003  -3.700460  ];

d = [-40.241827  -5723.355824  -10478.769093  -20308.350047  -28437.259095  -5647.392898  -10783.754952  -13550.347402  -6529.305162  -38.601475  -23.725317  -24.733523  -56.736969   ];

n=111;
rx = linspace(-200,200,n);
ry = linspace(-200,200,n);
rz = linspace(-10,10,n);

[x2,y2] = meshgrid(rx,ry);

conditions = {};
x=[];y=[];z=[];
eps = 0;
for i=1:length(d)
   z2 = (-abc(i,1)*x2 - abc(i,2)*y2 + d(i) )/abc(i,3); 
    for j=1:length(d)
        conditions{j} = abc(j,1)*x2 + abc(j,2)*y2 + abc(j,3)*z2 - d(i) > -eps;
    end
    region = conditions{1};
    for j=2:length(d)
        region = region & conditions{j};
    end
    
   x = [x;x2(region)];
   y = [y;y2(region)];
   z = [z;z2(region)];
end

figure;
region = ones(size(x2));
[k,av] = convhull(x,y,z, 'simplify',true);
trisurf(k,x,y,z, 'Facecolor','b'); %axis equal;
xlabel('x'); ylabel('Y'); zlabel('Z');
