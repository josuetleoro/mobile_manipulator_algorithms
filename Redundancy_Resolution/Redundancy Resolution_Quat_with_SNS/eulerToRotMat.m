function R=eulerToRotMat(phi,theta,psi,rep)

switch rep
    case 'ZYZ'
        cosPhi=cos(phi);
        sinPhi=sin(phi);
        cosTh=cos(theta);
        sinTh=sin(theta);
        cosPsi=cos(psi);
        sinPsi=sin(psi);
        
        R=[cosPhi*cosTh*cosPsi-sinPhi*sinPsi, -cosPhi*cosTh*sinPsi-sinPhi*cosPsi, cosPhi*sinTh;
            sinPhi*cosTh*cosPsi+cosPhi*sinPsi, -sinPhi*cosTh*sinPsi+cosPhi*cosPsi, sinPhi*sinTh;
            -sinTh*cosPsi, sinTh*sinPsi, cosTh];
    case 'ZYX'
        cosPhi=cos(phi);
        sinPhi=sin(phi);
        cosTh=cos(theta);
        sinTh=sin(theta);
        cosPsi=cos(psi);
        sinPsi=sin(psi);
        
        R=[cosPhi*cosTh, cosPhi*sinTh*sinPsi-sinPhi*cosPsi, cosPhi*sinTh*cosPsi+sinPhi*sinPsi;
           sinPhi*cosTh, sinPhi*sinTh*sinPsi+cosPhi*cosPsi, sinPhi*sinTh*cosPsi-cosPhi*sinPsi;
           -sinTh, cosTh*sinPsi, cosTh*cosPsi];
    otherwise
        error('The representation %s does not exist. Try ''ZYZ'' or ''ZYX''',rep)
end


end