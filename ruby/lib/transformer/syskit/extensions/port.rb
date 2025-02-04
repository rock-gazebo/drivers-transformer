module Transformer
    # Extension to Syskit::Port
    module PortExtension
        # Returns a transformation object that represents the transformation
        # this port is producing
        #
        # @return [Transformation,nil] the generated transformation, or nil if
        #   this port does not produce any. The transformation returned includes
        #   global frame names, not task-local ones.
        def produced_transformation
            component.find_transform_of_port(self)
        end

        # Returns the frame name in which the data on this port is expressed.
        #
        # @return [String,nil] the frame name, or nil if there are none. It
        #   returns global frame names, not task-local ones
        def associated_frame
            component.find_frame_of_port(self)
        end

        # Yields the ports connected to this port as well as their associated
        # frame
        #
        # It only yields the ports that do have a frame associated
        def each_frame_of_connected_ports
            return enum_for(__method__) unless block_given?

            each_concrete_connection do |other_port, _|
                if other_frame = other_port.associated_frame
                    yield other_port, other_frame
                end
            end
        end

        # Yields the ports connected to this port as well as their associated
        # transformation
        #
        # It only yields the ports for which there is a transformation
        # associated. However, the from or to field of the transformation
        # might be nil if the corresponding task-local frame is not assigned
        # yet
        def each_transform_of_connected_ports
            return enum_for(__method__) unless block_given?

            each_concrete_connection do |other_port, _|
                if other_transform = other_port.produced_transformation
                    if other_transform.from || other_transform.to
                        yield other_port, other_transform
                    end
                end
            end
        end
    end
end
