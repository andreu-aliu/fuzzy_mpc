% This function extracts the linear equations and functions from a fis object
function out = extract_fis(fis)

    % MF type
    out.type = fis.Inputs(1).MembershipFunctions(1).Type;
    
    % Get membership functions
    out.n_in = length(fis.Inputs);
    out.mf = {};
    for i = 1:out.n_in
        out.n_mf = length(fis.Inputs(i).MembershipFunctions);
        for j = 1:out.n_mf
            out.mf{i,j} = fis.Inputs(i).MembershipFunctions(j).Parameters;
        end
    end
    
    % Get linear models
    out.n_r = length(fis.Outputs(1).MembershipFunctions);
    out.A = {}; out.b = {};
    for k = 1:out.n_r
        m_function = fis.Outputs(1).MembershipFunctions(k);
        if(strcmp(m_function.Type, 'linear'))
            coefs = m_function.Parameters;
            out.A{k} = coefs(1:end-1);
            out.b{k} = coefs(end);
        end
    end
end