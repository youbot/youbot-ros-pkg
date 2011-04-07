function output=rotInertiaTensor(angle,inertiaTensor)
oMatrix=ang2orth(angle);
output=oMatrix'*inertiaTensor*oMatrix;
endfunction
