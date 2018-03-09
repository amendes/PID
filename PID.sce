function [a, u]=f()
    kp=2.5, ki=0.00025, kd=0.50;

    error_p=0, error_d=0, error_i=0;
    i=1, dt=1.0;
    //This bound value helps the software to achieve a close enough SP, set point
    bound=3;

    ref=40
    encoder_count=10;

    while(dt < 600)
        
        //DPI errors
        error_d = error_p;       
        error_p = (ref - encoder_count);
        //Pulling up the terms *dt*ki from the main Equation U(i) to the Integral error smoothes the switching at dt = 400
        error_i = error_i + error_p*dt*ki;

        
        //Equation of PID controller
       u(i) = kp*error_p + error_i + ((error_p - error_d)*kd)/dt;
       
       //Actuation of the PV of the system
        if(u(i)>bound)
            encoder_count = encoder_count + 2;
        end
        if(u(i)<-bound)
            encoder_count = encoder_count - 2;
        end
        if((u(i) < bound) & (u(i) > -bound) )
            encoder_count = encoder_count;
        end
        if(dt == 400)
            encoder_count = 100;
        end
        //PV vector for ploting purporses
        a(i) = encoder_count;
        
        i = i + 1;
        dt = dt + 1;
        
    end
    
        //Black line for PV and Blue line for CV
        plot2d([a u]);
        // PV - process variable; CV - Control Variable 
endfunction
