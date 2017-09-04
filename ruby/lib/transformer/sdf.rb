require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def load_sdf(model, exclude_models: [], &producer_resolver)
            model = ::SDF::Root.load(model, flatten: false)
            parse_sdf_root(model, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_root(sdf, exclude_models: [], &producer_resolver)
            sdf.each_world do |w|
                parse_sdf_world(w, exclude_models: exclude_models, &producer_resolver)
            end
            sdf.each_model do |m|
                next if exclude_models.include?(m.name) || exclude_models.include?(m)
                parse_sdf_model(m, "", "", &producer_resolver)
            end
        end

        def parse_sdf_world(sdf, exclude_models: [], &producer_resolver)
            world_full_name = sdf.full_name
            frames world_full_name
            sdf.each_model do |m|
                next if exclude_models.include?(m.name) || exclude_models.include?(m)
                parse_sdf_model(m, "", world_full_name, world_name: world_full_name, &producer_resolver)
            end
        end

        def sdf_append_name(prefix, name)
            if prefix.empty? then name
            else "#{prefix}::#{name}"
            end
        end

        def parse_sdf_model(sdf, prefix = "", parent_name = "", world_name: nil, &producer_resolver)
            full_name = sdf_append_name(prefix, sdf.name)
            frames full_name

            if !parent_name.empty? && sdf.each_joint.none? { |l| l.parent_link == ::SDF::Link::World || l.child_link == ::SDF::Link::World }
                if sdf.static?
                    static_transform(*sdf.pose, full_name => parent_name)
                else
                    example_transform(*sdf.pose, full_name => parent_name)
                end
            end

            parse_sdf_links_and_joints(sdf, full_name, full_name, world_name: world_name, &producer_resolver)
        end

        def sdf_link_pose_in_world(m)
            pose = Eigen::Isometry3.Identity
            while !m.kind_of?(::SDF::World)
                pose = m.pose * pose
                m = m.parent
            end
            pose
        end

        def parse_sdf_links_and_joints(sdf, prefix = "", parent_name = "", world_name: nil, &producer_resolver)
            submodel2model = Hash.new
            submodel2model[sdf] = sdf.pose
            sdf.each_model_with_name do |submodel, m_name|
                model_pose = submodel.pose

                m = submodel
                while m != sdf
                    m = m.parent
                    grandchild2child = model_pose
                    child2parent     = m.pose
                    model_pose = child2parent * grandchild2child
                end
                submodel2model[submodel] = model_pose
            end

            relative_link_names = Hash.new
            sdf.each_link_with_name do |l, l_name|
                relative_link_names[l] = l_name
            end

            world_link = ::SDF::Link::World

            if root_link = sdf.each_link.first
                static_transform(*root_link.pose, sdf_append_name(prefix, relative_link_names[root_link]) => parent_name)
            end

            sdf.each_joint_with_name do |j, j_name|
                parent = j.parent_link
                child  = j.child_link

                if parent == ::SDF::Link::World
                    child2model  = submodel2model[child.parent] * child.pose
                    child2parent = sdf_link_pose_in_world(child)
                elsif child == ::SDF::Link::World
                    child2model  = Eigen::Isometry3.Identity
                    child2parent = sdf_link_pose_in_world(parent).inverse
                else
                    parent2model = submodel2model[parent.parent] * parent.pose
                    child2model  = submodel2model[child.parent] * child.pose
                    child2parent = parent2model.inverse * child2model
                end
                joint2child  = j.pose
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
                    parent_frame_name = world_name 
                else
                    parent_frame_name = sdf_append_name(parent_name, relative_link_names[parent])
                end
                if world_name && (child == world_link)
                    child_frame_name = world_name
                else
                    child_frame_name  = sdf_append_name(parent_name, relative_link_names[child])
                end

                if upper == lower
                    static_transform child2parent, child_frame_name => parent_frame_name
                else
                    joint_pre  = sdf_append_name(parent_name, "#{j_name}_pre")
                    joint_post = sdf_append_name(parent_name, "#{j_name}_post")
                    register_joint(joint_post, joint_pre, j)
                    static_transform(joint2child, joint_post => child_frame_name)
                    static_transform(joint2parent, joint_pre => parent_frame_name)
                    if producer_resolver && (p = producer_resolver.call(j))
                        dynamic_transform p, joint_post => joint_pre
                    end
                    example_transform post2pre, joint_post => joint_pre
                end
            end
        end
    end
    Transformer::Configuration.include SDF
end
