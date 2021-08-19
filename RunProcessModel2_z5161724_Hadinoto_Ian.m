function Xnext=RunProcessModel2_z5161724_Hadinoto_Ian(X,k,v,w,dt) 
    Xnext=X+dt*[v(k)*cos(X(3));v(k)*sin(X(3));deg2rad(w(k))-X(4);0];
end