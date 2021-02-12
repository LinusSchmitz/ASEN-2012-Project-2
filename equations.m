function [locations] = equations(t, initials)
    x = initials(1);    %X position 
    z = initials(2);    %Z position 
    Vel = initials(3);  %Velocity 
    mass_R = initials(4);   %Mass
    V_air = initials(5);    %Volume of air
    theta = initials(6);    %Angle
    M_air = initials(7);    %Mass of the air

    %Housekeeping
    global g gamma R A_bol rho_airAmb Vol_bol P_gaug V_air0 M_air0 C_D P_amb rho_wat A_thro C_d T_air0;
    %misc calculations  
    q = 0.5 * rho_airAmb * Vel^2;
    Drag = q * C_D * A_bol;
    if ~V_air < Vol_bol
        P_end = P_gaug * (V_air0/Vol_bol)^gamma;
        p = P_end * (M_air/M_air0)^gamma;
    end
    
    % section of flight in water acceleration 
    if V_air < Vol_bol
        p = P_gaug * (V_air0/V_air) ^ gamma;
        dmass_Rdt = -C_d * A_thro * sqrt(2 * rho_wat * (p - P_amb));   %Rate of change of the mass of the rocket
        F = C_d * A_thro * 2 * (p - P_amb) ;    %This is the force equation for the water
        dV_airdt = C_d * A_thro * sqrt((2/rho_wat) * (P_gaug * ((V_air0/V_air) ^ gamma) - P_amb)); %Rate of change of volume of the air
        dM_airdt = 0;   %Change in mass of air. Only water leaving now 
    % section of flight in air acceleration 
    elseif p > P_amb
        T_end=T_air0*(V_air0/Vol_bol)^(gamma-1);
        rho=M_air/Vol_bol;
        P_crit=p*(2/(gamma+1))^(gamma/(gamma-1));
        T=p/(rho*R);
        if P_crit>P_amb
            pe=P_crit;
            Te=(2/(gamma+1))*T;
            rho_e=pe/(R*Te);
            Ve=sqrt(gamma*R*Te);
        else 
            pe=P_amb;
            Me=sqrt((2/(gamma-1))*(((p/P_amb)^((gamma-1)/gamma))-1));
            Te=T*(1+((gamma-1)/2)* Me^2);          %WARNING THIS MIGHT BE WRONG. MIGHT HAVE TO BE * INSTEAD OF /
            rho_e=pe/(R*Te);
            Ve=Me*sqrt(gamma*R*Te);
        end
        dM_airdt=-C_d*rho_e*A_thro*Ve;
        dmass_Rdt=dM_airdt;
        dV_airdt=0;
        F=-dM_airdt*Ve+(pe - P_amb)*A_thro;
        
    else 
        % section of flight in free fall 
        F=0;
        dmass_Rdt=0;
        dV_airdt=0;
        dM_airdt=0;
    end
    if Vel<1
        dthetadt=0;
    else 
        dthetadt=-(g*cos(theta))/Vel;
    end
    dzdt=Vel*sin(theta);
    dxdt=Vel*cos(theta);
    dVeldt=(F-Drag-(mass_R*g*sin(theta)))/mass_R;   %Rate of change of velocity
    
    locations = zeros(7,1);
    locations(1) = dxdt;        %Change in the x axis
    locations(2) = dzdt;        %Change in the z axis
    locations(3) = dVeldt;      %Rate of change of velocity of the rocket
    locations(4) = dmass_Rdt;   %Rate of change of the mass of the rocket
    locations(5) = dV_airdt;    %Rate of change of volume of air
    locations(6) = dthetadt;    %Rate of change of angle
    locations(7) = dM_airdt;    %Rate of change of mass of air 


end