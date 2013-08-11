msd = Model;
m = ureal('m',msd.m,'Percentage',msd.pm);
c = ureal('c',msd.c,'Percentage',msd.pc);
k = ureal('k',msd.k,'Percentage',msd.pk);

A = [-c/m -k/m; 1 0];
B = [1/m; 0];
C = [0 1];
D = 0;

sys = ss(A,B,C,D);