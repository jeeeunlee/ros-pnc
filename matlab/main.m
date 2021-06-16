abc = [  1.650113   1.651432   4.655667  
 504.376451   235.093295   1206.336218  
-0.051667   494.143165   1262.159320  
 832.823274  -124.990870   63.064702  
-1341.731168  -1.995103  -2481.959767  
-497.671505  -231.968209  -1002.133339  
 0.052935  -508.533771  -939.586637  
-555.705509   83.400262   409.205632  
 308.119843   0.457933   787.419566  
-1.582614  -1.583879  -2.506797  
 2.091197  -0.144267   2.331884  
-2.180389   0.150419  -2.038086  
 0.000002   0.000002  -3.700460   ];

d = [-6.708112  -953.873249  -1746.506742  -3384.617582  -4738.634577  -941.226097  -1797.432437  -2258.319465  -1088.157973  -6.433708  -3.953928  -4.122565  -56.736969  ];

n=111;
rx = linspace(-50,30,n);
ry = linspace(-50,50,n);
rz = linspace(-5,5,n);
[x,y,z] = meshgrid(rx, ry, rz);

conditions = {};
for i = 1:length(d)
    conditions{i} = abc(i,1)*x + abc(i,2)*y + abc(i,3)*z - d(i) > 0;
end

region = conditions{1};

for i=2:length(d)
   region = region & conditions{i}; 
end


% [k,av] = convhull(x(region),y(region),z(region));
% trisurf(k,x,y,z, 'Facecolor','cyan');

close all;
figure(1);
ss = repmat(10,numel(x(region)),1);
cc = [x(region)/30,y(region)/30,z(region)/5]; %repmat(1,numel(x(region)),1);
scatter3(x(region),y(region),z(region),ss,cc,'d','filled');
xlabel('X'); ylabel('Y'); zlabel('Z'); %axis equal;

figure(2);
srf = isosurface(x,y,z,region,0.99)
p=patch(srf)
set(p,'FaceColor','red','EdgeColor','none');
daspect([1,1,1])
view(3); 
camlight 
lighting gouraud
grid on