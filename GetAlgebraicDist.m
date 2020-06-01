% GetAlgeraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:* 
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end
