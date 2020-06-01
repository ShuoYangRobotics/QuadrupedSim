classdef TransitionStateProcessor
    properties
        total_time = 5;
        leg_num = 4;
        traj_pos_dim = 3;
        size_knot = 6;
        sw_knot_num = 1;
        st_knot_num = 6;
        co_knot_num = 1;
    end
    
   
    methods
        function obj = TransitionStateProcessor(param)
            obj.leg_num = param.leg_num;
            obj.total_time = param.total_time;
            obj.traj_pos_dim = param.traj_pos_dim;
            obj.sw_knot_num = param.sw_knot_num;
            obj.st_knot_num = param.st_knot_num;
            obj.co_knot_num = param.co_knot_num;
            obj.size_knot = param.size_knot_n;
        end
        
        % return or set the phase time
        function val = getPhaseTime(obj, state, leg_idx, is_value)
            idx = 2*(leg_idx-1) + 1;
            if (is_value == 0)
                val = [idx;idx+1];
            else
                val = [state(idx);state(idx+1)];
            end
        end
        function state = setPhaseTime(obj, state, val, leg_idx)
            idx = 2*(leg_idx-1) + 1;
            state(idx) = val(1);
            state(idx+1) = val(2);
        end
        % com pos time
        function val = getComPosTime(obj, state, knot_idx, is_value)
            idx = 2*obj.leg_num + (knot_idx-1) + 1;
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setComPosTime(obj, state, knot_idx, val)
            idx = 2*obj.leg_num + (knot_idx-1) + 1;
            state(idx) = val;
        end
        % com angle time
        function val = getComAngTime(obj, state, knot_idx, is_value)
            idx = 2*obj.leg_num + obj.co_knot_num + (knot_idx-1) + 1;
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setComAngTime(obj, state, knot_idx, val)
            idx = 2*obj.leg_num + obj.co_knot_num + (knot_idx-1) + 1;
            state(idx) = val;
        end
        % leg swing knot time
        function val = getLegSwingTime(obj, state, leg_idx, knot_idx, is_value)
            idx = 2*obj.leg_num + 2*obj.co_knot_num +...
                 (leg_idx - 1)*obj.sw_knot_num + (knot_idx - 1) + 1;
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setLegSwingTime(obj, state, val, leg_idx, knot_idx)
            idx = 2*obj.leg_num + 2*obj.co_knot_num +...
                 (leg_idx - 1)*obj.sw_knot_num + (knot_idx - 1) + 1;
            state(idx) = val;
        end
        % leg stance knot time
        function val = getLegStanceTime(obj, state, leg_idx, knot_idx, is_value)
            idx = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 (leg_idx - 1)*obj.st_knot_num*2 + (knot_idx - 1) + 1;
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setLegStanceTime(obj, state, val, leg_idx, knot_idx)
            idx = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 (leg_idx - 1)*obj.st_knot_num*2 + (knot_idx - 1) + 1;
            state(idx) = val;
        end
        % com pos knot 
        function val = getComPosKnot(obj, state, knot_idx, is_value)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 +...
                 obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        % function caller must make sure val has the right size
        function state = setComPosKnot(obj, state, val, knot_idx)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 +...
                 obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            state(idx) = val;
        end
        
        % com angle knot 
        function val = getComAngKnot(obj, state, knot_idx, is_value)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setComAngKnot(obj, state, val, knot_idx)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            state(idx) = val;
        end
        % leg swing knot 
        function val = getLegSwingKnot(obj, state, leg_idx, knot_idx, is_value)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.sw_knot_num*(leg_idx-1) + obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end 
        function state = setLegSwingKnot(obj, state, val, leg_idx, knot_idx)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.sw_knot_num*(leg_idx-1) + obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            state(idx) = val;
        end 
        % leg stance knot 
        function val = getLegStanceKnot(obj, state, leg_idx, knot_idx, is_value)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.leg_num*obj.sw_knot_num+...
                 obj.size_knot*obj.st_knot_num*2*(leg_idx-1) + obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            if (is_value == 0)
                val = idx;
            else
                val = state(idx);
            end
        end
        function state = setLegStanceKnot(obj, state, val, leg_idx, knot_idx)
            idx_start = 2*obj.leg_num + 2*obj.co_knot_num + obj.leg_num*obj.sw_knot_num +...
                 obj.leg_num*obj.st_knot_num*2 + obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.co_knot_num+...
                 obj.size_knot*obj.leg_num*obj.sw_knot_num+...
                 obj.size_knot*obj.st_knot_num*2*(leg_idx-1) + obj.size_knot*(knot_idx-1) + 1;
            idx = idx_start:idx_start+obj.size_knot-1; idx = idx';
            state(idx) = val;
        end
    end
    
    
end