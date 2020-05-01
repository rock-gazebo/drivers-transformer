require 'transformer/test'

module Transformer
    describe Configuration do
        before do
            @manager = Configuration.new
        end

        describe "#rename_frames" do
            describe "frame set" do
                it "applies the renames to the frame set" do
                    @manager.frames 'to_rename'
                    @manager.rename_frames 'to_rename' => 'test'
                    assert_equal ['test'], @manager.frames.to_a
                end
                it "does not modify frames that are not specified" do
                    @manager.frames 'to_rename'
                    @manager.rename_frames Hash.new
                    assert_equal ['to_rename'], @manager.frames.to_a
                end
            end

            describe "static transforms" do
                before do
                    @t = Eigen::Vector3.new(1, 2, 3)
                    @q = Eigen::Quaternion.new(1, 2, 3, 4)
                    @manager.static_transform @t, @q, 'to_rename_src' => 'to_rename_dst'
                end

                it "applies the renames to the static transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.transformation_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.transformation_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
            end

            describe "dynamic transforms" do
                before do
                    @producer = flexmock
                    @producer.should_receive(dup: @producer)
                    @manager.dynamic_transform @producer, 'to_rename_src' => 'to_rename_dst'
                end
                it "applies the renames to the dynamic transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.transformation_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @producer, transform.producer
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.transformation_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @producer, transform.producer
                end
            end

            describe "example transforms" do
                before do
                    @t = Eigen::Vector3.new(1, 2, 3)
                    @q = Eigen::Quaternion.new(1, 2, 3, 4)
                    @manager.example_transform @t, @q, 'to_rename_src' => 'to_rename_dst'
                end
                it "applies the renames to the example transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.example_transform_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.example_transform_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
            end
        end
    end
end


class TC_Transformer < Minitest::Test
    attr_reader :trsf, :transforms

    def conf
        trsf.conf
    end

    def setup
        super

        @trsf = Transformer::TransformationManager.new
        conf.frames "body", "servo_low", "servo_high", "laser", "camera", "camera_optical"

        @transforms = [
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"),
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "servo_high" => "laser"),
            conf.static_transform(Eigen::Vector3.new(0, 0, 0), "laser" => "camera")
        ]
    end

    def test_simple_transformation_chain
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_low" => "servo_high"))
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, false, false], chain.inversions)
    end

    def test_transformation_chain_with_inversions
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_high" => "servo_low"))
        chain = trsf.transformation_chain("body", "laser")
        assert_equal(transforms[0, 3], chain.links)
        assert_equal([false, true, false], chain.inversions)
    end

    def test_transformation_chain_with_loop_shortest_path
        transforms.insert(1, conf.dynamic_transform("dynamixel", "servo_high" => "servo_low"))
        transforms << conf.static_transform(Eigen::Vector3.new(0, 0, 0), "servo_high" => "body")
        chain = trsf.transformation_chain("body", "laser")
        assert_equal([transforms[4], transforms[2]], chain.links)
        assert_equal([true, false], chain.inversions)
    end

    def test_dup_isolates_the_sets_of_the_receiver_from_the_new_object
        trsf = Transformer::TransformationManager.new
        conf = trsf.conf
        conf.static_transform Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"
        conf.dynamic_transform 'dynamixel', "servo_high" => "servo_low"
        copy = conf.dup
        conf.clear
        assert_equal 2, copy.transforms.size
        assert_equal 3, copy.frames.size
    end
end

