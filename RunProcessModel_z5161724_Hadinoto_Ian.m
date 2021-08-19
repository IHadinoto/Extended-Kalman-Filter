function Xnext=RunProcessModel_z5161724_Hadinoto_Ian(X,k,v,wgb,dt) 
    Xnext=X+dt*[v(k)*cos(X(3));v(k)*sin(X(3));deg2rad(wgb(k))];
end