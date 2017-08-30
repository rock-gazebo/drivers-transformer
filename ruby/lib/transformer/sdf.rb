require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def load_sdf(model, exclude_models: [], &producer_resolver)
            model = ::SDF::Root.load(model, flatten: false)
            parse_sdf_root(model, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_root(sdf, exclude_models: [], &producer_resolver)
            parse_sdf(sdf, "", "", exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_world(sdf, exclude_models: [], &producer_resolver)
            frames sdf.full_name
            parse_sdf(sdf, "", sdf.full_name, world_name: sdf.full_name, exclude_models: exclude_models, &producer_resolver)
        end

        def sdf_append_name(prefix, name)
            if prefix.empty? then name
            else "#{prefix}::#{name}"
            end
        end

        def parse_sdf_model(sdf, prefix = "", parent_name = "", world_name: nil, exclude_models: [], &producer_resolver)
            full_name = sdf_append_name(prefix, sdf.name)
            frames full_name

            if !parent_name.empty?
                if sdf.static?
                    static_transform(*sdf.pose, sdf.name => parent_name)
                else
                    example_transform(*sdf.pose, sdf.name => parent_name)
                end
            end

            parse_sdf(sdf, full_name, full_name, world_name: world_name, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_links_and_joints(sdf, prefix = "", parent_name = "", world_name: nil, &producer_resolver)
            root_links = Hash.new
            relative_link_names = Hash.new
            sdf.each_link_with_name do |l, l_name|
                root_links[l_name] = l
                relative_link_names[l] = l_name
            end

            world_link = ::SDF::Link::World

            sdf.each_joint_with_name do |j|
                parent = j.parent_link
                child  = j.child_link
                root_links.delete(child.name)

                parent2model = parent.pose
                child2model  = child.pose
                joint2child  = j.pose
                child2parent = parent2model.inverse * child2model
                joint2parent = child2parent * joint2child

                if j.type != 'fixed'
                    axis_limit = j.axis.limit
                    upper = axis_limit.upper || 0
                    lower = axis_limit.lower || 0
                    axis = j.axis.xyz
                    if j.axis.use_parent_model_frame?
                        # The axis is expressed in the parent model frame ...
                        # Convert to joint frame
                        joint2model = child2model * joint2child
                        axis = joint2model.rotation.inverse * axis
                    end
                    post2pre = j.transform_for((upper + lower) / 2, nil, axis)
                end

                # Handle the special 'world' link
                if world_name && (parent == world_link)
                    parent = world_name 
                else
                    parent = sdf_append_name(parent_name, relative_link_names[parent])
                end
                if world_name && (child == world_link)
                    child = world_name
                else
                    child  = sdf_append_name(parent_name, relative_link_names[child])
                end

                if upper == lower
                    static_transform child2parent, child => parent
                else
                    joint_pre  = sdf_append_name(parent_name, "#{j.name}_pre")
                    joint_post = sdf_append_name(parent_name, "#{j.name}_post")
                    register_joint(joint_post, joint_pre, j)
                    static_transform(joint2child, joint_post => child)
                    static_transform(joint2parent, joint_pre => parent)
                    if producer_resolver && (p = producer_resolver.call(j))
                        dynamic_transform p, joint_post => joint_pre
                    end
                    example_transform post2pre, joint_post => joint_pre
                end
            end

            root_links.each_value do |l|
                static_transform(*l.pose, sdf_append_name(prefix, l.name) => parent_name)
            end
        end

        # @api private
        def parse_sdf(sdf, prefix, parent_name, world_name: nil, exclude_models: [], &producer_resolver)
            if sdf.respond_to?(:each_world)
                sdf.each_world { |w| parse_sdf_world(w, exclude_models: exclude_models, &producer_resolver) }
            end
            if sdf.respond_to?(:each_model)
                sdf.each_model do |m|
                    next if exclude_models.include?(m.name) || exclude_models.include?(m)
                    parse_sdf_model(m, prefix, parent_name, world_name: world_name, exclude_models: exclude_models, &producer_resolver)
                end
            end
            if sdf.respond_to?(:each_link)
                parse_sdf_links_and_joints(sdf, prefix, parent_name, world_name: world_name, &producer_resolver)
            end
        end
    end
    Transformer::Configuration.include SDF
end
