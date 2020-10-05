function plot_3d_surf(x,y,z,c)
dt = delaunayTriangulation(x',y');
tri = dt.ConnectivityList;
xi = dt.Points(:,1); 
yi = dt.Points(:,2); 
F = scatteredInterpolant(x',y',z');
zi = F(xi,yi) ;
trisurf(tri,xi,yi,zi, 'FaceColor',c, 'EdgeColor', 'None') 
end

