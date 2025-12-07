import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              Start Learning Physical AI 🤖
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function AdditionalInfoSection() {
  return (
    <section className={styles.infoSection}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className={styles.infoCard}>
              <h3>🤖 Physical AI</h3>
              <p>Learn how to build intelligent systems that interact with the physical world using ROS 2, Gazebo, and NVIDIA Isaac.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.infoCard}>
              <h3>🎓 Adaptive Learning</h3>
              <p>Personalized content that adapts to your skill level with AI-powered recommendations and assessments.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.infoCard}>
              <h3>💬 AI Assistant</h3>
              <p>Get instant help with concepts, code examples, and troubleshooting through our integrated RAG chatbot.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`AI-Native Textbook - ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI & Humanoid Robotics with interactive elements and AI-powered learning assistance">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <AdditionalInfoSection />
      </main>
    </Layout>
  );
}
