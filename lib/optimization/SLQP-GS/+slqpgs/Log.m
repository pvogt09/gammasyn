% Log class
classdef Log < handle
  
  % Class properties (private set access)
  properties (SetAccess = private, GetAccess = private)
   
    fields  % Cell array of property names to log
    iterate % Pointer to the Iterate object
    log     % Struct array of the optionally logged data 
    index   % Current index of log
    
  end
  
  % Class methods
  methods

    % Constructor
    function l = Log(i,z)
      
      l.fields  = i.log_fields;
      l.iterate = z;
      l.index   = 0;
      l.log     = [];
     
      if isempty(l.fields)
        return
      end
        
      for j = 1:length(l.fields)
        field = l.fields{j};
        try 
          % Exclude the follow class properties of Iterate from logging
          switch field
            case {'x_'}
              error('go to catch block');
          end
          value = getData(l.iterate,field);
          data.(field) = value;
        catch 
          error('SLQP-GS: Logging, field %s is invalid.',field);
        end
      end
      if isfield(data,'kkt')
          data.kkt = nan;
      end
      
      l.index = 1;
      l.log = repmat(data,1,i.iter_max+1);
       
    end
    
    function logData(l)
        
      if isempty(l.fields)
        return
      end
        
      l.index = l.index + 1;
      for j = 1:length(l.fields)
        field = l.fields{j};
        l.log(l.index).(field) = getData(l.iterate,field);
      end
      
    end
    
    function log = getLog(l)
        log = l.log(1:l.index);
    end
    
  end

end

function value = getData(z,field)
  switch field
    case 'x'
      value = z.x(:,1);
    case 'x_samples'
      value = z.x(:,2:end);
    case {'JE','JI'}
      value = z.(field)(:,:,1);
    case {'JE_samples','JI_samples'}
      value = z.(field(1:2))(:,:,2:end); 
    otherwise
      value = z.(field);
  end
end

