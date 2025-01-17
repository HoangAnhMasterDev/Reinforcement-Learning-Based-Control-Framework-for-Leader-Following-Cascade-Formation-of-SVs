function a = sig(beta,X)
    sig = [sign(X(1))*abs(X(1))^beta;
                sign(X(2))*abs(X(2))^beta;
                sign(X(3))*abs(X(3))^beta];
    a = sig;
end
